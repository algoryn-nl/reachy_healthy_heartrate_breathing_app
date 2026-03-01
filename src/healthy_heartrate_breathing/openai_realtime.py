import os
import base64
import random
import asyncio
import logging
from typing import Any, Final, Tuple, Literal, Optional
from decimal import Decimal
from pathlib import Path
from datetime import datetime

import numpy as np
from openai import AsyncOpenAI
from fastrtc import AdditionalOutputs, AsyncStreamHandler, wait_for_item, audio_to_int16
from numpy.typing import NDArray
from scipy.signal import resample
from websockets.exceptions import ConnectionClosedError

from healthy_heartrate_breathing.config import config
from healthy_heartrate_breathing.prompts import get_session_voice, get_session_instructions
from healthy_heartrate_breathing.env_utils import env_int, env_flag, env_float
from healthy_heartrate_breathing.audio_router import AudioRouter
from healthy_heartrate_breathing.tool_dispatcher import ToolDispatcher
from healthy_heartrate_breathing.tools.core_tools import (
    ToolDependencies,
    get_tool_specs,
    dispatch_tool_call,
)
from healthy_heartrate_breathing.transcript_handler import TranscriptHandler


logger = logging.getLogger(__name__)

OPEN_AI_INPUT_SAMPLE_RATE: Final[Literal[24000]] = 24000
OPEN_AI_OUTPUT_SAMPLE_RATE: Final[Literal[24000]] = 24000

# Cost tracking from usage data (pricing as of Feb 2026 https://openai.com/api/pricing/)
# Decimal avoids float precision loss during cumulative cost accumulation.
_1M = Decimal("1_000_000")
AUDIO_INPUT_COST_PER_1M = Decimal("32")
AUDIO_OUTPUT_COST_PER_1M = Decimal("64")
TEXT_INPUT_COST_PER_1M = Decimal("4")
TEXT_OUTPUT_COST_PER_1M = Decimal("16")
IMAGE_INPUT_COST_PER_1M = Decimal("5")


def _compute_response_cost(usage: Any) -> Decimal:
    """Compute dollar cost from a response usage object."""
    inp = getattr(usage, "input_token_details", None)
    out = getattr(usage, "output_token_details", None)
    cost = Decimal(0)
    if inp:
        cost += (getattr(inp, "audio_tokens", 0) or 0) * AUDIO_INPUT_COST_PER_1M / _1M
        cost += (getattr(inp, "text_tokens", 0) or 0) * TEXT_INPUT_COST_PER_1M / _1M
        cost += (getattr(inp, "image_tokens", 0) or 0) * IMAGE_INPUT_COST_PER_1M / _1M
    if out:
        cost += (getattr(out, "audio_tokens", 0) or 0) * AUDIO_OUTPUT_COST_PER_1M / _1M
        cost += (getattr(out, "text_tokens", 0) or 0) * TEXT_OUTPUT_COST_PER_1M / _1M
    return cost


class OpenaiRealtimeHandler(AsyncStreamHandler):
    """An OpenAI realtime handler for fastrtc Stream."""

    def __init__(self, deps: ToolDependencies, gradio_mode: bool = False, instance_path: Optional[str] = None):
        """Initialize the handler."""
        super().__init__(
            expected_layout="mono",
            output_sample_rate=OPEN_AI_OUTPUT_SAMPLE_RATE,
            input_sample_rate=OPEN_AI_INPUT_SAMPLE_RATE,
        )

        # Override typing of the sample rates to match OpenAI's requirements
        self.output_sample_rate: Literal[24000] = OPEN_AI_OUTPUT_SAMPLE_RATE
        self.input_sample_rate: Literal[24000] = OPEN_AI_INPUT_SAMPLE_RATE

        self.deps = deps

        self.connection: Any = None
        self.output_queue: "asyncio.Queue[Tuple[int, NDArray[np.int16]] | AdditionalOutputs]" = asyncio.Queue()

        self.last_activity_time = asyncio.get_event_loop().time()
        self.start_time = asyncio.get_event_loop().time()
        self.is_idle_tool_call = False
        self.gradio_mode = gradio_mode
        self.instance_path = instance_path
        # Track how the API key was provided (env vs textbox) and its value
        self._key_source: Literal["env", "textbox"] = "env"
        self._provided_api_key: str | None = None

        # Internal lifecycle flags
        self._shutdown_requested: bool = False
        self._connected_event: asyncio.Event = asyncio.Event()

        # Cost tracking (Decimal avoids float precision loss over many responses)
        self.cumulative_cost: Decimal = Decimal(0)

        # Latest sensor readings (updated after each mmWave tool call, polled by /sensor endpoint)
        self.sensor_state: dict[str, Any] = {}

        # Idle scanning policy (delegates to IdlePolicy)
        from healthy_heartrate_breathing.idle_policy import IdlePolicy

        self.idle_policy = IdlePolicy(
            interval_s=env_float("HEALTHY_MM_WAVE_IDLE_DEFAULT_INTERVAL_S", 15.0, min_value=1.0),
            probe_interval_s=env_float("HEALTHY_MM_WAVE_IDLE_PROBE_INTERVAL_S", 40.0, min_value=1.0),
            probe_duration_s=env_float("HEALTHY_MM_WAVE_IDLE_PROBE_DURATION_S", 5.0, min_value=0.5),
            misses_before_sweep=env_int("HEALTHY_MM_WAVE_MISSES_BEFORE_SWEEP", 3, min_value=1),
            sweep_cooldown_s=env_float("HEALTHY_MM_WAVE_SWEEP_COOLDOWN_S", 150.0, min_value=1.0),
            post_focus_quiet_s=env_float("HEALTHY_MM_WAVE_POST_FOCUS_QUIET_S", 45.0, min_value=0.0),
            errors_before_suppression=env_int("HEALTHY_MM_WAVE_ERRORS_BEFORE_SUPPRESSION", 3, min_value=1),
            error_backoff_s=env_float("HEALTHY_MM_WAVE_ERROR_BACKOFF_S", 120.0, min_value=1.0),
        )
        logger.info(
            "mmWave idle policy: probe_interval=%.1fs, probe_duration=%.1fs, "
            "misses_before_sweep=%d, sweep_cooldown=%.1fs, post_focus_quiet=%.1fs",
            self.idle_policy.probe_interval_s,
            self.idle_policy.probe_duration_s,
            self.idle_policy.misses_before_sweep,
            self.idle_policy.sweep_cooldown_s,
            self.idle_policy.post_focus_quiet_s,
        )

        # Light-context policy orchestration (delegates to LightOrchestrator)
        from healthy_heartrate_breathing.light_orchestrator import LightOrchestrator

        _light_user_id = (os.getenv("HEALTHY_LIGHT_CONTEXT_USER_ID", "default") or "default").strip() or "default"
        self.light_orchestrator = LightOrchestrator(
            enabled=env_flag("HEALTHY_AUTO_LIGHT_CONTEXT_ENABLED", True),
            analytics_enabled=env_flag("HEALTHY_LIGHT_ANALYTICS_ENABLED", True),
            user_id=_light_user_id,
            prefers_dim=env_flag("HEALTHY_LIGHT_PREFERS_DIM", False),
            light_sensitive=env_flag("HEALTHY_LIGHT_SENSITIVE", False),
            allow_wellness_nudges=env_flag("HEALTHY_LIGHT_ALLOW_WELLNESS_NUDGES", True),
            day_start_hour=env_int("HEALTHY_LIGHT_DAY_START_HOUR", 7, min_value=0, max_value=23),
            night_start_hour=env_int("HEALTHY_LIGHT_NIGHT_START_HOUR", 20, min_value=0, max_value=23),
            low_lux_threshold=env_float("HEALTHY_LIGHT_LOW_LUX_THRESHOLD", 40.0, min_value=0.0),
            baseline_alpha=env_float("HEALTHY_LIGHT_BASELINE_ALPHA", 0.15, min_value=0.01, max_value=1.0),
            baseline_min_samples=env_int("HEALTHY_LIGHT_BASELINE_MIN_SAMPLES", 5, min_value=1),
            baseline_max_age_days=env_int("HEALTHY_LIGHT_BASELINE_MAX_AGE_DAYS", 90, min_value=1),
            baseline_path=self._resolve_runtime_data_path("light_context_baseline.json"),
            analytics_path=self._resolve_runtime_data_path("light_context_analytics.jsonl"),
            analytics_max_bytes=env_int("HEALTHY_LIGHT_ANALYTICS_MAX_BYTES", 5_000_000, min_value=1024),
        )
        logger.info(
            "Light-context policy: enabled=%s, analytics=%s, user_id=%s, baseline=%s",
            self.light_orchestrator.enabled,
            self.light_orchestrator.analytics_enabled,
            self.light_orchestrator.user_id,
            self.light_orchestrator.baseline_path,
        )

    def copy(self) -> "OpenaiRealtimeHandler":
        """Create a copy of the handler."""
        return OpenaiRealtimeHandler(self.deps, self.gradio_mode, self.instance_path)

    def _has_tool(self, name: str) -> bool:
        """Return whether a given tool name is available in this session."""
        return any(spec.get("name") == name for spec in get_tool_specs())

    def _replace_sensor_state(self, state: dict[str, Any]) -> None:
        """Replace sensor_state with a fresh snapshot (clear stale keys)."""
        self.sensor_state.clear()
        self.sensor_state.update(state)

    def _touch_activity(self) -> None:
        """Update the last activity timestamp."""
        self.last_activity_time = asyncio.get_event_loop().time()
        logger.debug("last activity time updated to %s", self.last_activity_time)

    def _resolve_runtime_data_path(self, filename: str) -> Path:
        """Resolve a writable path for runtime policy/analytics data."""
        base = Path(self.instance_path) if self.instance_path else Path.cwd()
        return base / filename

    async def apply_personality(self, profile: str | None) -> str:
        """Apply a new personality (profile) at runtime if possible.

        - Updates the global config's selected profile for subsequent calls.
        - If a realtime connection is active, sends a session.update with the
          freshly resolved instructions so the change takes effect immediately.

        Returns a short status message for UI feedback.
        """
        try:
            # Update the in-process config value and env
            from healthy_heartrate_breathing.config import config as _config
            from healthy_heartrate_breathing.config import set_custom_profile

            set_custom_profile(profile)
            logger.info(
                "Set custom profile to %r (config=%r)", profile, getattr(_config, "REACHY_MINI_CUSTOM_PROFILE", None)
            )

            try:
                instructions = get_session_instructions()
                voice = get_session_voice()
            except Exception as e:
                logger.error("Failed to resolve personality content: %s", e)
                return f"Failed to apply personality: {e}"

            # Attempt a live update first, then force a full restart to ensure it sticks
            if self.connection is not None:
                try:
                    await self.connection.session.update(
                        session={
                            "type": "realtime",
                            "instructions": instructions,
                            "audio": {"output": {"voice": voice}},
                        },
                    )
                    logger.info("Applied personality via live update: %s", profile or "built-in default")
                except Exception as e:
                    logger.warning("Live update failed; will restart session: %s", e)

                # Force a real restart to guarantee the new instructions/voice
                try:
                    await self._restart_session()
                    return "Applied personality and restarted realtime session."
                except Exception as e:
                    logger.warning("Failed to restart session after apply: %s", e)
                    return "Applied personality. Will take effect on next connection."
            else:
                logger.info(
                    "Applied personality recorded: %s (no live connection; will apply on next session)",
                    profile or "built-in default",
                )
                return "Applied personality. Will take effect on next connection."
        except Exception as e:
            logger.error("Error applying personality '%s': %s", profile, e)
            return f"Failed to apply personality: {e}"

    async def start_up(self) -> None:
        """Start the handler with minimal retries on unexpected websocket closure."""
        openai_api_key = config.OPENAI_API_KEY
        if self.gradio_mode and not openai_api_key:
            # api key was not found in .env or in the environment variables
            await self.wait_for_args()  # type: ignore[no-untyped-call]
            args = list(self.latest_args)
            textbox_api_key = args[3] if len(args[3]) > 0 else None
            if textbox_api_key is not None:
                openai_api_key = textbox_api_key
                self._key_source = "textbox"
                self._provided_api_key = textbox_api_key
            else:
                openai_api_key = config.OPENAI_API_KEY
        else:
            if not openai_api_key or not openai_api_key.strip():
                # In headless console mode, LocalStream now blocks startup until the key is provided.
                # However, unit tests may invoke this handler directly with a stubbed client.
                # To keep tests hermetic without requiring a real key, fall back to a placeholder.
                logger.warning("OPENAI_API_KEY missing. Proceeding with a placeholder (tests/offline).")
                openai_api_key = "DUMMY"

        self.client = AsyncOpenAI(api_key=openai_api_key)

        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            try:
                await self._run_realtime_session()
                # Normal exit from the session, stop retrying
                return
            except ConnectionClosedError as e:
                # Abrupt close (e.g., "no close frame received or sent") → retry
                logger.warning("Realtime websocket closed unexpectedly (attempt %d/%d): %s", attempt, max_attempts, e)
                if attempt < max_attempts:
                    # exponential backoff with jitter
                    base_delay = 2 ** (attempt - 1)  # 1s, 2s, 4s, 8s, etc.
                    jitter = random.uniform(0, 0.5)
                    delay = base_delay + jitter
                    logger.info("Retrying in %.1f seconds...", delay)
                    await asyncio.sleep(delay)
                    continue
                raise
            finally:
                # never keep a stale reference
                self.connection = None
                try:
                    self._connected_event.clear()
                except Exception:
                    logger.debug("Failed to clear connected event in start_up", exc_info=True)

    async def _restart_session(self) -> None:
        """Force-close the current session and start a fresh one in background.

        Does not block the caller while the new session is establishing.
        """
        try:
            if self.connection is not None:
                try:
                    await self.connection.close()
                except Exception:
                    logger.debug("Failed to close connection during restart", exc_info=True)
                finally:
                    self.connection = None

            # Ensure we have a client (start_up must have run once)
            if getattr(self, "client", None) is None:
                logger.warning("Cannot restart: OpenAI client not initialized yet.")
                return

            # Fire-and-forget new session and wait briefly for connection
            try:
                self._connected_event.clear()
            except Exception:
                logger.debug("Failed to clear connected event during restart", exc_info=True)
            asyncio.create_task(self._run_realtime_session(), name="openai-realtime-restart")
            try:
                await asyncio.wait_for(self._connected_event.wait(), timeout=5.0)
                logger.info("Realtime session restarted and connected.")
            except asyncio.TimeoutError:
                logger.warning("Realtime session restart timed out; continuing in background.")
        except Exception as e:
            logger.warning("_restart_session failed: %s", e)

    async def _run_realtime_session(self) -> None:
        """Establish and manage a single realtime session."""
        async with self.client.realtime.connect(model=config.MODEL_NAME) as conn:
            try:
                await conn.session.update(
                    session={
                        "type": "realtime",
                        "instructions": get_session_instructions(),
                        "audio": {
                            "input": {
                                "format": {
                                    "type": "audio/pcm",
                                    "rate": self.input_sample_rate,
                                },
                                "transcription": {"model": "gpt-4o-transcribe", "language": "en"},
                                "turn_detection": {
                                    "type": "server_vad",
                                    "interrupt_response": True,
                                },
                            },
                            "output": {
                                "format": {
                                    "type": "audio/pcm",
                                    "rate": self.output_sample_rate,
                                },
                                "voice": get_session_voice(),
                            },
                        },
                        "tools": get_tool_specs(),  # type: ignore[typeddict-item]
                        "tool_choice": "auto",
                    },
                )
                logger.info(
                    "Realtime session initialized with profile=%r voice=%r",
                    getattr(config, "REACHY_MINI_CUSTOM_PROFILE", None),
                    get_session_voice(),
                )
                self._persist_api_key_if_needed()
            except Exception:
                logger.exception("Realtime session.update failed; aborting startup")
                return

            logger.info("Realtime session updated successfully")

            # Build callbacks that close over `conn` and `self.output_queue`
            async def _enqueue_output(payload: dict) -> None:
                await self.output_queue.put(AdditionalOutputs(payload))

            async def _enqueue_audio(sample_rate: int, audio_array: Any) -> None:
                await self.output_queue.put((sample_rate, audio_array))

            async def _send_tool_result(call_id: str, output_json: str) -> None:
                await conn.conversation.item.create(
                    item={
                        "type": "function_call_output",
                        "call_id": call_id,
                        "output": output_json,
                    },
                )

            async def _create_response(instructions: str) -> None:
                await conn.response.create(
                    response={"instructions": instructions},
                )

            async def _create_message(item: dict) -> None:
                await conn.conversation.item.create(item=item)

            # Instantiate handlers
            transcript = TranscriptHandler(
                debounce_delay=0.5,
                enqueue_output=_enqueue_output,
            )

            dispatcher = ToolDispatcher(
                idle_policy=self.idle_policy,
                light_orchestrator=self.light_orchestrator,
                has_tool=self._has_tool,
                dispatch_tool=lambda name, args: dispatch_tool_call(name, args, self.deps),
                send_tool_result=_send_tool_result,
                create_response=_create_response,
                create_message=_create_message,
                enqueue_output=_enqueue_output,
                get_camera_frame=lambda: (
                    self.deps.camera_worker.get_latest_frame() if self.deps.camera_worker is not None else None
                ),
                head_wobbler_reset=(self.deps.head_wobbler.reset if self.deps.head_wobbler is not None else None),
                timeout_s=env_float("HEALTHY_TOOL_DISPATCH_TIMEOUT_S", 30.0, min_value=1.0),
                on_sensor_update=self._replace_sensor_state,
            )
            self._dispatcher = dispatcher

            audio = AudioRouter(
                output_sample_rate=self.output_sample_rate,
                enqueue_audio=_enqueue_audio,
                feed_head_wobbler=(self.deps.head_wobbler.feed if self.deps.head_wobbler is not None else None),
                on_activity=self._touch_activity,
            )

            # Store transcript handler for shutdown cleanup
            self._transcript_handler = transcript

            # Event loop
            self.connection = conn
            try:
                self._connected_event.set()

                async for event in self.connection:
                    logger.debug("OpenAI event: %s", event.type)

                    if event.type == "input_audio_buffer.speech_started":
                        if hasattr(self, "_clear_queue") and callable(self._clear_queue):
                            self._clear_queue()
                        if self.deps.head_wobbler is not None:
                            self.deps.head_wobbler.reset()
                        self.deps.movement_manager.set_listening(True)

                    elif event.type == "input_audio_buffer.speech_stopped":
                        self.deps.movement_manager.set_listening(False)

                    elif event.type == "response.done":
                        response = getattr(event, "response", None)
                        usage = getattr(response, "usage", None) if response else None
                        if usage:
                            cost = _compute_response_cost(usage)
                            self.cumulative_cost += cost
                            logger.debug("Cost: $%.4f | Cumulative: $%.4f", cost, self.cumulative_cost)
                        else:
                            logger.warning("No usage data available for cost tracking")

                    elif event.type == "conversation.item.input_audio_transcription.partial":
                        await transcript.on_partial(event.transcript)

                    elif event.type == "conversation.item.input_audio_transcription.completed":
                        await transcript.on_user_completed(event.transcript)

                    elif event.type in ("response.audio_transcript.done", "response.output_audio_transcript.done"):
                        await transcript.on_assistant_done(event.transcript)

                    elif event.type in ("response.audio.delta", "response.output_audio.delta"):
                        await audio.on_audio_delta(event.delta)

                    elif event.type == "response.function_call_arguments.done":
                        tool_name = getattr(event, "name", None)
                        args_json_str = getattr(event, "arguments", None)
                        call_id = getattr(event, "call_id", None)
                        if not isinstance(tool_name, str) or not isinstance(args_json_str, str):
                            logger.error("Invalid tool call: tool_name=%s, args=%s", tool_name, args_json_str)
                            continue
                        dispatcher.dispatch(
                            tool_name=tool_name,
                            args_json=args_json_str,
                            call_id=call_id,
                            is_idle=self.is_idle_tool_call,
                        )
                        if self.is_idle_tool_call:
                            self.is_idle_tool_call = False

                    elif event.type == "error":
                        err = getattr(event, "error", None)
                        msg = getattr(err, "message", str(err) if err else "unknown error")
                        code = getattr(err, "code", "")
                        logger.error("Realtime error [%s]: %s (raw=%s)", code, msg, err)
                        if code not in ("input_audio_buffer_commit_empty", "conversation_already_has_active_response"):
                            await _enqueue_output({"role": "assistant", "content": f"[error] {msg}"})
            finally:
                self.connection = None
                self._connected_event.clear()

    # Microphone receive
    async def receive(self, frame: Tuple[int, NDArray[np.int16]]) -> None:
        """Receive audio frame from the microphone and send it to the OpenAI server.

        Handles both mono and stereo audio formats, converting to the expected
        mono format for OpenAI's API. Resamples if the input sample rate differs
        from the expected rate.

        Args:
            frame: A tuple containing (sample_rate, audio_data).

        """
        if not self.connection:
            return

        input_sample_rate, audio_frame = frame

        # Reshape if needed
        if audio_frame.ndim == 2:
            # Scipy channels last convention
            if audio_frame.shape[1] > audio_frame.shape[0]:
                audio_frame = audio_frame.T
            # Multiple channels -> Mono channel
            if audio_frame.shape[1] > 1:
                audio_frame = audio_frame[:, 0]

        # Resample if needed
        if self.input_sample_rate != input_sample_rate:
            audio_frame = resample(audio_frame, int(len(audio_frame) * self.input_sample_rate / input_sample_rate))
            # scipy.signal.resample returns float64; fastrtc expects float32 or int16
            audio_frame = audio_frame.astype(np.float32)

        # Cast if needed
        audio_frame = audio_to_int16(audio_frame)

        # Send to OpenAI (guard against races during reconnect)
        try:
            audio_message = base64.b64encode(audio_frame.tobytes()).decode("utf-8")
            await self.connection.input_audio_buffer.append(audio=audio_message)
        except Exception as e:
            logger.debug("Dropping audio frame: connection not ready (%s)", e)
            return

    async def emit(self) -> Tuple[int, NDArray[np.int16]] | AdditionalOutputs | None:
        """Emit audio frame to be played by the speaker."""
        # sends to the stream the stuff put in the output queue by the openai event handler
        # This is called periodically by the fastrtc Stream

        # Handle idle
        now = asyncio.get_event_loop().time()
        has_mmwave = self._has_tool("mmWave")
        idle_interval = self.idle_policy.probe_interval_s if has_mmwave else self.idle_policy.interval_s
        idle_duration = now - self.last_activity_time
        is_moving = not self.deps.movement_manager.is_idle()
        if idle_duration > idle_interval and not is_moving:
            if has_mmwave and not self.idle_policy.should_trigger(idle_duration, is_moving=False, now=now):
                self.last_activity_time = now
                return None
            try:
                await self.send_idle_signal(idle_duration)
            except Exception as e:
                self.is_idle_tool_call = False
                logger.warning("Idle signal skipped (connection closed?): %s", e)
                return None

            self.last_activity_time = now  # avoid repeated resets

        return await wait_for_item(self.output_queue)  # type: ignore[no-any-return]

    async def shutdown(self) -> None:
        """Shutdown the handler."""
        self._shutdown_requested = True
        # Cancel any pending transcript debounce
        if hasattr(self, "_transcript_handler"):
            await self._transcript_handler.cancel_pending()

        # Cancel any in-flight tool dispatch
        if hasattr(self, "_dispatcher"):
            await self._dispatcher.cancel()

        # Flush any dirty light baseline to disk
        self.light_orchestrator.flush()

        if self.connection:
            try:
                await self.connection.close()
            except ConnectionClosedError as e:
                logger.debug(f"Connection already closed during shutdown: {e}")
            except Exception as e:
                logger.debug(f"connection.close() ignored: {e}")
            finally:
                self.connection = None

        # Clear any remaining items in the output queue
        while not self.output_queue.empty():
            try:
                self.output_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    def format_timestamp(self) -> str:
        """Format current timestamp with date, time, and elapsed seconds."""
        loop_time = asyncio.get_event_loop().time()  # monotonic
        elapsed_seconds = loop_time - self.start_time
        dt = datetime.now()  # wall-clock
        return f"[{dt.strftime('%Y-%m-%d %H:%M:%S')} | +{elapsed_seconds:.1f}s]"

    async def get_available_voices(self) -> list[str]:
        """Try to discover available voices for the configured realtime model.

        Attempts to retrieve model metadata from the OpenAI Models API and look
        for any keys that might contain voice names. Falls back to a curated
        list known to work with realtime if discovery fails.
        """
        # Conservative fallback list with default first
        fallback = [
            "cedar",
            "alloy",
            "aria",
            "ballad",
            "verse",
            "sage",
            "coral",
        ]
        try:
            # Best effort discovery; safe-guarded for unexpected shapes
            model = await self.client.models.retrieve(config.MODEL_NAME)
            # Try common serialization paths
            raw = None
            for attr in ("model_dump", "to_dict"):
                fn = getattr(model, attr, None)
                if callable(fn):
                    try:
                        raw = fn()
                        break
                    except Exception:
                        logger.debug("Model serialization via %s failed", attr, exc_info=True)
            if raw is None:
                try:
                    raw = dict(model)
                except Exception:
                    logger.debug("dict() conversion of model object failed", exc_info=True)
                    raw = None
            # Scan for voice candidates
            candidates: set[str] = set()

            def _collect(obj: object) -> None:
                try:
                    if isinstance(obj, dict):
                        for k, v in obj.items():
                            kl = str(k).lower()
                            if "voice" in kl and isinstance(v, (list, tuple)):
                                for item in v:
                                    if isinstance(item, str):
                                        candidates.add(item)
                                    elif isinstance(item, dict) and "name" in item and isinstance(item["name"], str):
                                        candidates.add(item["name"])
                            else:
                                _collect(v)
                    elif isinstance(obj, (list, tuple)):
                        for it in obj:
                            _collect(it)
                except Exception:
                    logger.debug("Voice candidate collection failed for object", exc_info=True)

            if isinstance(raw, dict):
                _collect(raw)
            # Ensure default present and stable order
            voices = sorted(candidates) if candidates else fallback
            if "cedar" not in voices:
                voices = ["cedar", *[v for v in voices if v != "cedar"]]
            return voices
        except Exception:
            logger.debug("Voice discovery failed; returning fallback list", exc_info=True)
            return fallback

    async def send_idle_signal(self, idle_duration: float) -> None:
        """Send an idle signal to the openai server."""
        logger.debug("Sending idle signal")
        self.is_idle_tool_call = True
        has_mmwave = self._has_tool("mmWave")
        if has_mmwave:
            now = asyncio.get_event_loop().time()
            sweep_allowed = self.idle_policy.sweep_allowed(now)
            strategy = self.idle_policy.build_strategy_message(sweep_allowed)
            sweep_flag = "true" if sweep_allowed else "false"
            scan_only = self.idle_policy.suggest_scan_only
            idle_mode = "scan" if scan_only else "locate_and_measure"
            timestamp_msg = (
                f"[Idle time update: {self.format_timestamp()} - No activity for {idle_duration:.1f}s] "
                "Do one calm wellness scan cycle."
            )
            idle_instructions = (
                "You MUST respond with function calls only - no speech or text. "
                f"Call mmWave exactly once with mode='{idle_mode}', duration_s={self.idle_policy.probe_duration_s}, "
                f"{'sweep_if_unseen=' + sweep_flag + '. ' if not scan_only else ''}Then stop. "
                f"Current strategy: {strategy}. "
                "Do not call dance, play_emotion, or sweep_look for this idle cycle."
            )
            logger.info(
                "Idle schedule: mmWave probe (misses=%d/%d, sweep_allowed=%s, scan_only=%s)",
                self.idle_policy.consecutive_misses,
                self.idle_policy.misses_before_sweep,
                sweep_allowed,
                scan_only,
            )
        else:
            timestamp_msg = (
                f"[Idle time update: {self.format_timestamp()} - No activity for {idle_duration:.1f}s] "
                "You've been idle for a while. Feel free to get creative - dance, show an emotion, "
                "look around, do nothing, or just be yourself!"
            )
            idle_instructions = (
                "You MUST respond with function calls only - no speech or text. "
                "Choose appropriate actions for idle behavior."
            )
        if not self.connection:
            logger.debug("No connection, cannot send idle signal")
            return
        await self.connection.conversation.item.create(
            item={
                "type": "message",
                "role": "user",
                "content": [{"type": "input_text", "text": timestamp_msg}],
            },
        )
        await self.connection.response.create(
            response={
                "instructions": idle_instructions,
                "tool_choice": "required",
            },
        )

    def _persist_api_key_if_needed(self) -> None:
        """Persist the API key into `.env` inside `instance_path/` when appropriate.

        - Only runs in Gradio mode when key came from the textbox and is non-empty.
        - Only saves if `self.instance_path` is not None.
        - Writes `.env` to `instance_path/.env` (does not overwrite if it already exists).
        - If `instance_path/.env.example` exists, copies its contents while overriding OPENAI_API_KEY.
        """
        try:
            if not self.gradio_mode:
                logger.warning("Not in Gradio mode; skipping API key persistence.")
                return

            if self._key_source != "textbox":
                logger.info("API key not provided via textbox; skipping persistence.")
                return

            key = (self._provided_api_key or "").strip()
            if not key:
                logger.warning("No API key provided via textbox; skipping persistence.")
                return
            if self.instance_path is None:
                logger.warning("Instance path is None; cannot persist API key.")
                return

            # Update the current process environment for downstream consumers
            try:
                import os

                os.environ["OPENAI_API_KEY"] = key
            except Exception:
                logger.debug("Best-effort os.environ OPENAI_API_KEY assignment failed", exc_info=True)

            target_dir = Path(self.instance_path)
            env_path = target_dir / ".env"
            if env_path.exists():
                # Respect existing user configuration
                logger.info(".env already exists at %s; not overwriting.", env_path)
                return

            example_path = target_dir / ".env.example"
            content_lines: list[str] = []
            if example_path.exists():
                try:
                    content = example_path.read_text(encoding="utf-8")
                    content_lines = content.splitlines()
                except Exception as e:
                    logger.warning("Failed to read .env.example at %s: %s", example_path, e)

            # Replace or append the OPENAI_API_KEY line
            replaced = False
            for i, line in enumerate(content_lines):
                if line.strip().startswith("OPENAI_API_KEY="):
                    content_lines[i] = f"OPENAI_API_KEY={key}"
                    replaced = True
                    break
            if not replaced:
                content_lines.append(f"OPENAI_API_KEY={key}")

            # Ensure file ends with newline
            final_text = "\n".join(content_lines) + "\n"
            env_path.write_text(final_text, encoding="utf-8")
            logger.info("Created %s and stored OPENAI_API_KEY for future runs.", env_path)
        except Exception as e:
            # Never crash the app for QoL persistence; just log.
            logger.warning("Could not persist OPENAI_API_KEY to .env: %s", e)
