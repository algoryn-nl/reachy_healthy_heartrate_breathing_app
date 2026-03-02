# Design: Decompose `launch()` into smaller methods (PY-MED-7)

## Problem

`LocalStream.launch()` in `console.py` is ~100 lines with four distinct responsibilities mixed into one method: instance env loading, HuggingFace key fallback, settings UI / key wait, and async session lifecycle. This makes it hard to read, test, and modify individual phases.

## Approach: Flat private methods

Extract four private methods on `LocalStream`. `launch()` becomes a short orchestrator.

### New methods

| Method | Source lines | Responsibility |
|--------|-------------|----------------|
| `_load_instance_env()` | 334–358 | Load instance `.env`, update config (API key + profile) |
| `_acquire_api_key_from_hf()` | 361–373 | HuggingFace `gradio_client` key download fallback |
| `_wait_for_api_key() -> bool` | 375–386 | Mount settings UI, block if key missing; returns `False` on interrupt |
| `_run_async_session()` | 393–422 | Async coroutine: mount personality routes, create tasks, gather, shutdown |

### Resulting `launch()`

```python
def launch(self) -> None:
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

### Constraints

- Pure extraction refactor: no behavior changes
- No new classes, files, or dependencies
- Only `console.py` modified
- Existing tests remain valid without changes

## Alternatives considered

- **Startup helper class**: Clean separation but over-engineered for 4 sequential steps.
- **Module-level functions**: Would break encapsulation since methods use `self._instance_path`, `self._persist_api_key()`, etc.
