"""Entrypoint for the Reachy Mini conversation app."""

import os
import sys
import time
import asyncio
import argparse
import threading
from typing import Any, Dict, List, Optional
from pathlib import Path

import gradio as gr
from fastapi import FastAPI
from fastrtc import Stream
from gradio.utils import get_space

from reachy_mini import ReachyMini, ReachyMiniApp
from healthy_heartrate_breathing.utils import (
    parse_args,
    setup_logger,
    handle_vision_stuff,
    log_connection_troubleshooting,
)


def update_chatbot(chatbot: List[Dict[str, Any]], response: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Update the chatbot with AdditionalOutputs."""
    chatbot.append(response)
    return chatbot


def main() -> None:
    """Entrypoint for the Reachy Mini conversation app."""
    args, _ = parse_args()
    run(args)


def run(
    args: argparse.Namespace,
    robot: ReachyMini | None = None,
    app_stop_event: Optional[threading.Event] = None,
    settings_app: Optional[FastAPI] = None,
    instance_path: Optional[str] = None,
) -> None:
    """Run the Reachy Mini conversation app."""
    # Putting these dependencies here makes the dashboard faster to load when the conversation app is installed
    try:
        from healthy_heartrate_breathing.moves import MovementManager
        from healthy_heartrate_breathing.console import LocalStream
        from healthy_heartrate_breathing.openai_realtime import OpenaiRealtimeHandler
        from healthy_heartrate_breathing.tools.core_tools import ToolDependencies
        from healthy_heartrate_breathing.audio.head_wobbler import HeadWobbler
    except Exception as e:
        # ConfigError (or other import-time failures) from config/prompts validation
        print(f"Fatal: {e}", file=sys.stderr)
        sys.exit(1)

    logger = setup_logger(args.debug)
    logger.info("Starting Reachy Mini Conversation App")

    if args.no_camera and args.head_tracker is not None:
        logger.warning("Head tracking disabled: --no-camera flag is set. Remove --no-camera to enable head tracking.")

    if robot is None:
        try:
            robot_kwargs = {}
            if args.robot_name is not None:
                robot_kwargs["robot_name"] = args.robot_name

            logger.info("Initializing ReachyMini (SDK will auto-detect appropriate backend)")
            robot = ReachyMini(**robot_kwargs)

        except TimeoutError as e:
            logger.error(f"Connection timeout: Failed to connect to Reachy Mini daemon. Details: {e}")
            log_connection_troubleshooting(logger, args.robot_name)
            sys.exit(1)

        except ConnectionError as e:
            logger.error(f"Connection failed: Unable to establish connection to Reachy Mini. Details: {e}")
            log_connection_troubleshooting(logger, args.robot_name)
            sys.exit(1)

        except Exception as e:
            logger.error(f"Unexpected error during robot initialization: {type(e).__name__}: {e}")
            logger.error("Please check your configuration and try again.")
            sys.exit(1)

    # Auto-enable Gradio in simulation mode (both MuJoCo for deamon and mockup-sim for desktop app)
    status = robot.client.get_status()
    is_simulation = status.get("simulation_enabled", False) or status.get("mockup_sim_enabled", False)

    if is_simulation and not args.gradio:
        logger.info("Simulation mode detected. Automatically enabling gradio flag.")
        args.gradio = True

    camera_worker, _, vision_manager = handle_vision_stuff(args, robot)

    movement_manager = MovementManager(
        current_robot=robot,
        camera_worker=camera_worker,
    )

    head_wobbler = HeadWobbler(set_speech_offsets=movement_manager.set_speech_offsets)

    deps = ToolDependencies(
        reachy_mini=robot,
        movement_manager=movement_manager,
        camera_worker=camera_worker,
        vision_manager=vision_manager,
        head_wobbler=head_wobbler,
    )
    current_file_path = os.path.dirname(os.path.abspath(__file__))
    logger.debug(f"Current file absolute path: {current_file_path}")
    chatbot = gr.Chatbot(
        type="messages",
        resizable=True,
        avatar_images=(
            os.path.join(current_file_path, "images", "user_avatar.png"),
            os.path.join(current_file_path, "images", "reachymini_avatar.png"),
        ),
    )
    logger.debug(f"Chatbot avatar images: {chatbot.avatar_images}")

    handler = OpenaiRealtimeHandler(deps, gradio_mode=args.gradio, instance_path=instance_path)
    deps.trend_analyzer = handler.trend_analyzer

    stream_manager: gr.Blocks | LocalStream | None = None

    if args.gradio:
        api_key_textbox = gr.Textbox(
            label="OPENAI API Key",
            type="password",
            value=os.getenv("OPENAI_API_KEY") if not get_space() else "",
        )

        stream = Stream(
            handler=handler,
            mode="send-receive",
            modality="audio",
            additional_inputs=[chatbot, api_key_textbox],
            additional_outputs=[chatbot],
            additional_outputs_handler=update_chatbot,
            ui_args={"title": "Healthy Heartrate Breathing"},
        )
        stream_manager = stream.ui

        # Inject dashboard panels below the default Stream UI.
        # dashboard.css variables are defined inline; dashboard.js/CSS loaded as static files.
        static_dir = Path(__file__).parent / "static"
        _dashboard_css = (static_dir / "dashboard.css").read_text(encoding="utf-8")
        _dashboard_js = (static_dir / "dashboard.js").read_text(encoding="utf-8")

        with stream_manager:
            # CSS variable root + dashboard styles (inline to avoid static-file routing issues)
            gr.HTML(
                f"""<style>
:root {{
  --bg: #060b1a;
  --bg-2: #071023;
  --panel: rgba(11, 18, 36, 0.8);
  --border: rgba(255, 255, 255, 0.08);
  --text: #eaf2ff;
  --muted: #9fb6d7;
  --ok: #4ce0b3;
  --warn: #ffb547;
  --error: #ff5c70;
  --accent: #45c4ff;
  --accent-2: #5ef0c1;
  --shadow: 0 20px 70px rgba(0, 0, 0, 0.45);
}}
{_dashboard_css}
</style>""",
                visible=True,
            )

            # Chart.js CDN (required before dashboard.js)
            gr.HTML(
                '<script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>'
                '<script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns@3/dist/'
                'chartjs-adapter-date-fns.bundle.min.js"></script>',
                visible=True,
            )

            # Dashboard panels
            gr.HTML(
                """<div class="dashboard-row">
  <div id="vitals-card" class="vitals-card">
    <div class="vitals-row">
      <div class="vitals-item">
        <span class="vitals-label">Heart Rate</span>
        <span class="vitals-value vitals-inactive" id="hr-value">--</span>
        <span class="vitals-unit">bpm</span>
      </div>
      <div class="vitals-item">
        <span class="vitals-label">Breathing</span>
        <span class="vitals-value vitals-inactive" id="br-value">--</span>
        <span class="vitals-unit">bpm</span>
      </div>
    </div>
    <div class="vitals-meta">
      <span id="state-badge" class="state-badge state-none">--</span>
      <span id="target-count">Targets: --</span>
      <span id="lux-value">Lux: --</span>
    </div>
    <div class="vitals-status">
      <span id="status-dot" class="status-dot"></span>
      <span id="status-text">Connecting...</span>
    </div>
  </div>
  <div id="radar-panel" class="radar-container">
    <canvas id="radar-canvas" width="300" height="300"></canvas>
  </div>
</div>""",
                visible=True,
            )

            gr.HTML(
                """<div id="chart-panel" class="chart-container">
  <canvas id="vitals-chart"></canvas>
</div>""",
                visible=True,
            )

            # Dashboard JS (inline, runs after DOM elements are rendered)
            gr.HTML(f"<script>{_dashboard_js}</script>", visible=True)

        if not settings_app:
            app = FastAPI()
        else:
            app = settings_app

        # Mount live sensor WebSocket and vitals history REST endpoint
        from starlette.websockets import WebSocket, WebSocketDisconnect

        @app.websocket("/ws/sensor")
        async def ws_sensor(websocket: WebSocket) -> None:
            await websocket.accept()
            handler.sensor_broadcaster.connect(websocket)
            try:
                while True:
                    await websocket.receive_text()  # keep-alive; ignore client messages
            except WebSocketDisconnect:
                handler.sensor_broadcaster.disconnect(websocket)

        @app.get("/api/vitals/history")
        async def vitals_history(hours: int = 4) -> dict:
            return {"rows": handler.vitals_store.query(hours=min(hours, 24))}

        app = gr.mount_gradio_app(app, stream.ui, path="/")
    else:
        # In headless mode, wire settings_app + instance_path to console LocalStream
        stream_manager = LocalStream(
            handler,
            robot,
            settings_app=settings_app,
            instance_path=instance_path,
        )

    # Each async service → its own thread/loop
    movement_manager.start()
    head_wobbler.start()
    if camera_worker:
        camera_worker.start()
    if vision_manager:
        vision_manager.start()

    def poll_stop_event() -> None:
        """Poll the stop event to allow graceful shutdown."""
        if app_stop_event is not None:
            app_stop_event.wait()

        logger.info("App stop event detected, shutting down...")
        try:
            stream_manager.close()
        except Exception as e:
            logger.error(f"Error while closing stream manager: {e}")

    if app_stop_event:
        threading.Thread(target=poll_stop_event, daemon=True).start()

    try:
        stream_manager.launch()
    except KeyboardInterrupt:
        logger.info("Keyboard interruption in main thread... closing server.")
    finally:
        movement_manager.stop()
        head_wobbler.stop()
        if camera_worker:
            camera_worker.stop()
        if vision_manager:
            vision_manager.stop()

        # Ensure media is explicitly closed before disconnecting
        try:
            robot.media.close()
        except Exception as e:
            logger.debug(f"Error closing media during shutdown: {e}")

        # prevent connection to keep alive some threads
        robot.client.disconnect()
        time.sleep(1)
        logger.info("Shutdown complete.")


class HealthyHeartrateBreathing(ReachyMiniApp):  # type: ignore[misc]
    """Reachy Mini Apps entry point for the conversation app."""

    custom_app_url = "http://0.0.0.0:7860/"
    dont_start_webserver = False

    def run(self, reachy_mini: ReachyMini, stop_event: threading.Event) -> None:
        """Run the Reachy Mini conversation app."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        args, _ = parse_args()

        # is_wireless = reachy_mini.client.get_status()["wireless_version"]
        # args.head_tracker = None if is_wireless else "mediapipe"

        instance_path = self._get_instance_path().parent
        run(
            args,
            robot=reachy_mini,
            app_stop_event=stop_event,
            settings_app=self.settings_app,
            instance_path=instance_path,
        )


if __name__ == "__main__":
    app = HealthyHeartrateBreathing()
    try:
        app.wrapped_run()
    except KeyboardInterrupt:
        app.stop()
