# launch() Decomposition Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Decompose `LocalStream.launch()` into four focused private methods (PY-MED-7).

**Architecture:** Pure extraction refactor — move code blocks into named methods, replace originals with calls. No behavior changes.

**Tech Stack:** Python 3.12, ruff, mypy, pytest

---

### Task 1: Extract `_load_instance_env()`

**Files:**
- Modify: `src/healthy_heartrate_breathing/console.py:325-422`

**Step 1: Add `_load_instance_env()` method before `launch()`**

Insert this method between `_init_settings_ui_if_needed()` (ends line 323) and `launch()` (starts line 325):

```python
def _load_instance_env(self) -> None:
    """Load the instance ``.env`` file and update config (API key + profile)."""
    if not self._instance_path:
        return
    try:
        from dotenv import load_dotenv

        from healthy_heartrate_breathing.config import set_custom_profile

        env_path = Path(self._instance_path) / ".env"
        if env_path.exists():
            load_dotenv(dotenv_path=str(env_path), override=True)
            # Update config with newly loaded values
            new_key = os.getenv("OPENAI_API_KEY", "").strip()
            if new_key:
                try:
                    config.OPENAI_API_KEY = new_key
                except Exception:
                    logger.warning("Failed to set config.OPENAI_API_KEY from instance .env", exc_info=True)
            if LOCKED_PROFILE is None:
                new_profile = os.getenv("REACHY_MINI_CUSTOM_PROFILE")
                if new_profile is not None:
                    try:
                        set_custom_profile(new_profile.strip() or None)
                    except Exception:
                        logger.debug("Best-effort profile update from instance .env failed", exc_info=True)
    except Exception:
        logger.debug("Instance .env loading failed; continuing with defaults", exc_info=True)
```

**Step 2: Replace the corresponding block in `launch()` with a call**

In `launch()`, replace lines 333–358 (the `# Try to load an existing instance .env` block) with:

```python
self._load_instance_env()
```

**Step 3: Verify**

Run: `uv run ruff check src/healthy_heartrate_breathing/console.py && uv run ruff format --check src/healthy_heartrate_breathing/console.py && uv run mypy src/healthy_heartrate_breathing/console.py`
Expected: no errors

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/console.py
git commit -m "refactor: extract _load_instance_env() from launch()"
```

---

### Task 2: Extract `_acquire_api_key_from_hf()`

**Files:**
- Modify: `src/healthy_heartrate_breathing/console.py`

**Step 1: Add `_acquire_api_key_from_hf()` method after `_load_instance_env()`**

```python
def _acquire_api_key_from_hf(self) -> None:
    """Attempt to download an OpenAI API key from HuggingFace as a fallback."""
    logger.info("OPENAI_API_KEY not set, attempting to download from HuggingFace...")
    try:
        from gradio_client import Client

        client = Client("HuggingFaceM4/gradium_setup", verbose=False)
        key, status = client.predict(api_name="/claim_b_key")
        if key and key.strip():
            logger.info("Successfully downloaded API key from HuggingFace")
            self._persist_api_key(key)
    except Exception as e:
        logger.warning(f"Failed to download API key from HuggingFace: {e}")
```

**Step 2: Replace the corresponding block in `launch()` with a call**

Replace the `# If key is still missing, try to download` block with:

```python
if not (config.OPENAI_API_KEY and str(config.OPENAI_API_KEY).strip()):
    self._acquire_api_key_from_hf()
```

**Step 3: Verify**

Run: `uv run ruff check src/healthy_heartrate_breathing/console.py && uv run ruff format --check src/healthy_heartrate_breathing/console.py && uv run mypy src/healthy_heartrate_breathing/console.py`
Expected: no errors

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/console.py
git commit -m "refactor: extract _acquire_api_key_from_hf() from launch()"
```

---

### Task 3: Extract `_wait_for_api_key()`

**Files:**
- Modify: `src/healthy_heartrate_breathing/console.py`

**Step 1: Add `_wait_for_api_key()` method after `_acquire_api_key_from_hf()`**

```python
def _wait_for_api_key(self) -> bool:
    """Mount settings UI and block until API key is available.

    Returns ``True`` if the key is available (immediately or after waiting),
    ``False`` if interrupted.
    """
    self._init_settings_ui_if_needed()
    if not (config.OPENAI_API_KEY and str(config.OPENAI_API_KEY).strip()):
        logger.warning("OPENAI_API_KEY not found. Open the app settings page to enter it.")
        try:
            self._api_key_event.wait()
        except KeyboardInterrupt:
            logger.info("Interrupted while waiting for API key.")
            return False
    return True
```

**Step 2: Replace the settings UI init + key wait blocks in `launch()`**

Replace the `# Always expose settings UI` comment through the end of the key-wait block with:

```python
if not self._wait_for_api_key():
    return
```

**Step 3: Verify**

Run: `uv run ruff check src/healthy_heartrate_breathing/console.py && uv run ruff format --check src/healthy_heartrate_breathing/console.py && uv run mypy src/healthy_heartrate_breathing/console.py`
Expected: no errors

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/console.py
git commit -m "refactor: extract _wait_for_api_key() from launch()"
```

---

### Task 4: Extract `_run_async_session()` and finalize `launch()`

**Files:**
- Modify: `src/healthy_heartrate_breathing/console.py`

**Step 1: Add `_run_async_session()` async method after `_wait_for_api_key()`**

```python
async def _run_async_session(self) -> None:
    """Run the async session: personality routes, handler, record/play loops."""
    loop = asyncio.get_running_loop()
    self._asyncio_loop = loop  # type: ignore[assignment]
    try:
        if self._settings_app is not None:
            mount_personality_routes(
                self._settings_app,
                self.handler,
                lambda: self._asyncio_loop,
                persist_personality=self._persist_personality,
                get_persisted_personality=self._read_persisted_personality,
            )
    except Exception:
        logger.warning("Failed to mount personality routes", exc_info=True)
    self._tasks = [
        asyncio.create_task(self.handler.start_up(), name="openai-handler"),
        asyncio.create_task(self.record_loop(), name="stream-record-loop"),
        asyncio.create_task(self.play_loop(), name="stream-play-loop"),
    ]
    try:
        await asyncio.gather(*self._tasks)
    except asyncio.CancelledError:
        logger.info("Tasks cancelled during shutdown")
    finally:
        await self.handler.shutdown()
```

**Step 2: Replace the inner `runner()` closure and `asyncio.run()` call in `launch()`**

Replace the `async def runner()` block and `asyncio.run(runner())` with:

```python
asyncio.run(self._run_async_session())
```

**Step 3: Verify final `launch()` reads cleanly**

The resulting `launch()` should be:

```python
def launch(self) -> None:
    """Start the recorder/player and run the async processing loops."""
    self._stop_event.clear()
    self._load_instance_env()
    if not (config.OPENAI_API_KEY and str(config.OPENAI_API_KEY).strip()):
        self._acquire_api_key_from_hf()
    if not self._wait_for_api_key():
        return
    self._robot.media.start_recording()
    self._robot.media.start_playing()
    time.sleep(1)
    asyncio.run(self._run_async_session())
```

**Step 4: Run full checks**

Run: `uv run ruff check src/healthy_heartrate_breathing/console.py && uv run ruff format --check src/healthy_heartrate_breathing/console.py && uv run mypy src/healthy_heartrate_breathing/console.py`
Expected: no errors

Run: `uv run pytest tests/ --ignore=tests/vision -x -q`
Expected: all tests pass

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/console.py
git commit -m "refactor: extract _run_async_session(), finalize launch() decomposition (PY-MED-7)"
```

---

### Task 5: Update documentation

**Files:**
- Modify: `docs/TODO.md`

**Step 1: Mark PY-MED-7 as done in TODO.md**

Change `- [ ] **PY-MED-7**:` to `- [x] **PY-MED-7**: Decompose launch() into smaller methods in console.py — extracted _load_instance_env(), _acquire_api_key_from_hf(), _wait_for_api_key(), _run_async_session() (2026-03-02)`

**Step 2: Commit**

```bash
git add docs/TODO.md
git commit -m "docs: mark PY-MED-7 as done"
```
