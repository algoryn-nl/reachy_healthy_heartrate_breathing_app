"""Tests for the shared env_utils module."""
# ruff: noqa: D103


def test_coerce_bool_true_values():
    from healthy_heartrate_breathing.env_utils import coerce_bool

    for val in [True, "1", "true", "True", "yes", "YES", "on", "ON"]:
        assert coerce_bool(val, False) is True, f"Expected True for {val!r}"


def test_coerce_bool_false_values():
    from healthy_heartrate_breathing.env_utils import coerce_bool

    for val in [False, "0", "false", "False", "no", "NO", "off", "OFF"]:
        assert coerce_bool(val, True) is False, f"Expected False for {val!r}"


def test_coerce_bool_default():
    from healthy_heartrate_breathing.env_utils import coerce_bool

    assert coerce_bool(None, True) is True
    assert coerce_bool("garbage", False) is False
    assert coerce_bool(42, True) is True  # non-bool, non-str -> default


def test_coerce_float_basic():
    from healthy_heartrate_breathing.env_utils import coerce_float

    assert coerce_float(3.14, 0.0) == 3.14
    assert coerce_float("2.5", 0.0) == 2.5
    assert coerce_float(10, 0.0) == 10.0


def test_coerce_float_none_and_invalid():
    from healthy_heartrate_breathing.env_utils import coerce_float

    assert coerce_float(None, 5.0) == 5.0
    assert coerce_float("bad", 5.0) == 5.0
    assert coerce_float(None, None) is None


def test_coerce_float_nonneg():
    from healthy_heartrate_breathing.env_utils import coerce_float_nonneg

    assert coerce_float_nonneg(-5.0, 1.0) == 0.0
    assert coerce_float_nonneg("3.0", 1.0) == 3.0
    assert coerce_float_nonneg("bad", 7.0) == 7.0


def test_coerce_int_basic():
    from healthy_heartrate_breathing.env_utils import coerce_int

    assert coerce_int(42, 0) == 42
    assert coerce_int("7", 0) == 7
    assert coerce_int(3.9, 0) == 3


def test_coerce_int_default():
    from healthy_heartrate_breathing.env_utils import coerce_int

    assert coerce_int(None, 10) == 10
    assert coerce_int("bad", 10) == 10


def test_coerce_ms():
    from healthy_heartrate_breathing.env_utils import coerce_ms

    assert coerce_ms(250, 100) == 250
    assert coerce_ms(0, 100) == 1  # clamped to min 1
    assert coerce_ms(-5, 100) == 1
    assert coerce_ms("bad", 100) == 100


def test_env_flag(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_flag

    monkeypatch.setenv("TEST_FLAG", "true")
    assert env_flag("TEST_FLAG", False) is True

    monkeypatch.setenv("TEST_FLAG", "0")
    assert env_flag("TEST_FLAG", True) is False

    monkeypatch.delenv("TEST_FLAG", raising=False)
    assert env_flag("TEST_FLAG", True) is True


def test_env_flag_invalid(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_flag

    monkeypatch.setenv("TEST_FLAG", "maybe")
    assert env_flag("TEST_FLAG", False) is False


def test_env_float(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_float

    monkeypatch.setenv("TEST_FLOAT", "3.14")
    assert env_float("TEST_FLOAT", 0.0) == 3.14

    monkeypatch.delenv("TEST_FLOAT", raising=False)
    assert env_float("TEST_FLOAT", 9.9) == 9.9


def test_env_float_clamping(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_float

    monkeypatch.setenv("TEST_FLOAT", "-10.0")
    assert env_float("TEST_FLOAT", 5.0, min_value=0.0) == 0.0

    monkeypatch.setenv("TEST_FLOAT", "999.0")
    assert env_float("TEST_FLOAT", 5.0, max_value=100.0) == 100.0


def test_env_float_invalid(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_float

    monkeypatch.setenv("TEST_FLOAT", "not_a_number")
    assert env_float("TEST_FLOAT", 42.0) == 42.0


def test_env_int(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_int

    monkeypatch.setenv("TEST_INT", "7")
    assert env_int("TEST_INT", 0) == 7

    monkeypatch.delenv("TEST_INT", raising=False)
    assert env_int("TEST_INT", 99) == 99


def test_env_int_clamping(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_int

    monkeypatch.setenv("TEST_INT", "-5")
    assert env_int("TEST_INT", 0, min_value=0) == 0

    monkeypatch.setenv("TEST_INT", "200")
    assert env_int("TEST_INT", 0, max_value=100) == 100


def test_env_int_float_string(monkeypatch):
    from healthy_heartrate_breathing.env_utils import env_int

    monkeypatch.setenv("TEST_INT", "3.7")
    assert env_int("TEST_INT", 0) == 3  # int(float("3.7")) == 3


def test_extract_lux_from_mmwave_result():
    from healthy_heartrate_breathing.env_utils import extract_lux_from_mmwave_result

    # measure path
    assert extract_lux_from_mmwave_result({"measure": {"latest_light": {"lux": 42.5}}}) == 42.5

    # scan path
    assert extract_lux_from_mmwave_result({"scan": {"latest_light": {"lux": 10.0}}}) == 10.0

    # light_summary path
    assert extract_lux_from_mmwave_result({"scan": {"light_summary": {"latest_lux": 7}}}) == 7.0

    # measure.light_summary fallback
    assert extract_lux_from_mmwave_result({"measure": {"light_summary": {"latest_lux": 99}}}) == 99.0

    # measure preferred over scan when both present
    assert extract_lux_from_mmwave_result(
        {
            "measure": {"latest_light": {"lux": 50.0}},
            "scan": {"latest_light": {"lux": 30.0}},
        }
    ) == 50.0

    # missing
    assert extract_lux_from_mmwave_result({}) is None
    assert extract_lux_from_mmwave_result(None) is None
    assert extract_lux_from_mmwave_result({"measure": {}}) is None

    # latest_light present but lux is None — should fall through to summary
    assert extract_lux_from_mmwave_result(
        {"scan": {"latest_light": {"lux": None}, "light_summary": {"latest_lux": 22.0}}}
    ) == 22.0


def test_extract_lux_realistic_locate_and_measure():
    """Regression: extraction against a realistic locate_and_measure response."""
    from healthy_heartrate_breathing.env_utils import extract_lux_from_mmwave_result

    result = {
        "serial_port": "/dev/cu.usbmodem1101",
        "mode": "locate_and_measure",
        "status": "ok",
        "scan": {
            "targets_seen": 3,
            "latest_target": {"cluster": 0, "x": 0.5, "y": 0.1, "r": 0.8, "bearing": 0.12, "v": 0.0},
            "recent_targets": [{"cluster": 0, "x": 0.5, "y": 0.1, "r": 0.8, "bearing": 0.12, "v": 0.0}],
            "latest_light": {"t_ms": 4200, "valid": True, "lux": 185.3},
            "light_samples": [{"t_ms": 1000, "lux": 180.0}, {"t_ms": 4200, "lux": 185.3}],
            "state": "STILL_NEAR",
            "telemetry": [],
            "light_summary": {"samples": 2, "valid_samples": 2, "latest_lux": 185.3, "avg_lux": 182.65, "min_lux": 180.0, "max_lux": 185.3},
        },
        "measure": {
            "attempts": 5,
            "latest_bio": {"hr": 72, "br": 16, "allowed": 1, "valid": 1},
            "latest_light": {"t_ms": 18500, "valid": True, "lux": 190.7},
            "light_samples": [{"t_ms": 10000, "lux": 188.0}, {"t_ms": 18500, "lux": 190.7}],
            "state": {"state": "RESTING_VITALS"},
            "bio_messages": [],
            "valid_bio": {"heart_rate_bpm": 72, "breath_rate_bpm": 16, "state": 5, "hr_new": 1, "br_new": 1},
            "success": True,
            "light_summary": {"samples": 2, "valid_samples": 2, "latest_lux": 190.7, "avg_lux": 189.35, "min_lux": 188.0, "max_lux": 190.7},
        },
    }

    # Should prefer measure.latest_light.lux (most recent reading)
    assert extract_lux_from_mmwave_result(result) == 190.7


def test_extract_lux_realistic_scan_only():
    """Regression: extraction against a realistic scan-only response."""
    from healthy_heartrate_breathing.env_utils import extract_lux_from_mmwave_result

    result = {
        "serial_port": "/dev/cu.usbmodem1101",
        "mode": "scan",
        "status": "scan_done",
        "scan": {
            "targets_seen": 0,
            "latest_target": None,
            "recent_targets": [],
            "latest_light": {"t_ms": 7800, "valid": True, "lux": 42.1},
            "light_samples": [{"t_ms": 3000, "lux": 40.5}, {"t_ms": 7800, "lux": 42.1}],
            "state": "NO_TARGET",
            "telemetry": [],
            "light_summary": {"samples": 2, "valid_samples": 2, "latest_lux": 42.1, "avg_lux": 41.3, "min_lux": 40.5, "max_lux": 42.1},
        },
    }

    # No measure phase — falls through to scan.latest_light.lux
    assert extract_lux_from_mmwave_result(result) == 42.1


def test_extract_lux_realistic_no_light_sensor():
    """Regression: extraction returns None when firmware has no light sensor."""
    from healthy_heartrate_breathing.env_utils import extract_lux_from_mmwave_result

    result = {
        "serial_port": "/dev/cu.usbmodem1101",
        "mode": "locate_and_measure",
        "status": "ok",
        "scan": {
            "targets_seen": 2,
            "latest_target": {"cluster": 1, "x": 0.3, "y": 0.2, "r": 0.6, "bearing": 0.1, "v": 0.0},
            "recent_targets": [],
            "latest_light": None,
            "light_samples": [],
            "state": "MOVING",
            "telemetry": [],
            "light_summary": {"samples": 0, "valid_samples": 0, "latest_lux": None, "avg_lux": None, "min_lux": None, "max_lux": None},
        },
        "measure": {
            "attempts": 3,
            "latest_bio": None,
            "latest_light": None,
            "light_samples": [],
            "state": None,
            "bio_messages": [],
            "valid_bio": None,
            "success": False,
            "light_summary": {"samples": 0, "valid_samples": 0, "latest_lux": None, "avg_lux": None, "min_lux": None, "max_lux": None},
        },
    }

    assert extract_lux_from_mmwave_result(result) is None
