const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function fetchWithTimeout(url, options = {}, timeoutMs = 2000) {
  const controller = new AbortController();
  const id = setTimeout(() => controller.abort(), timeoutMs);
  try {
    return await fetch(url, { ...options, signal: controller.signal });
  } finally {
    clearTimeout(id);
  }
}

async function waitForStatus(timeoutMs = 15000) {
  const loadingText = document.querySelector("#loading p");
  let attempts = 0;
  const deadline = Date.now() + timeoutMs;
  while (true) {
    attempts += 1;
    try {
      const url = new URL("/status", window.location.origin);
      url.searchParams.set("_", Date.now().toString());
      const resp = await fetchWithTimeout(url, {}, 2000);
      if (resp.ok) return await resp.json();
    } catch (e) {}
    if (loadingText) {
      loadingText.textContent = attempts > 8 ? "Starting backend\u2026" : "Loading\u2026";
    }
    if (Date.now() >= deadline) return null;
    await sleep(500);
  }
}

async function validateKey(key) {
  const body = { openai_api_key: key };
  const resp = await fetch("/validate_api_key", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await resp.json().catch(() => ({}));
  if (!resp.ok) {
    throw new Error(data.error || "validation_failed");
  }
  return data;
}

async function saveKey(key) {
  const body = { openai_api_key: key };
  const resp = await fetch("/openai_api_key", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!resp.ok) {
    const data = await resp.json().catch(() => ({}));
    throw new Error(data.error || "save_failed");
  }
  return await resp.json();
}

function show(el, flag) {
  el.classList.toggle("hidden", !flag);
}

/* ---- Sensor Dashboard ---- */

const SENSOR_POLL_MS = 3000;
const STATE_LABELS = {
  NO_TARGET: "No Target",
  MULTI_TARGET: "Multi-Target",
  PRESENT_FAR: "Present (Far)",
  MOVING: "Moving",
  STILL_NEAR: "Still (Near)",
  RESTING_VITALS: "Resting",
};

function formatAge(epochSec) {
  if (!epochSec) return "--";
  const ageSec = Math.round((Date.now() / 1000) - epochSec);
  if (ageSec < 5) return "just now";
  if (ageSec < 60) return ageSec + "s ago";
  if (ageSec < 3600) return Math.floor(ageSec / 60) + "m ago";
  return Math.floor(ageSec / 3600) + "h ago";
}

function updateSensorUI(data) {
  const panel = document.getElementById("sensor-panel");
  const chip = document.getElementById("sensor-chip");
  if (!panel) return;

  if (!data || !data.available) {
    show(panel, false);
    return;
  }

  show(panel, true);

  // Chip status
  const deviceState = data.device_state || null;
  if (deviceState === "RESTING_VITALS" || deviceState === "STILL_NEAR") {
    chip.textContent = "Active";
    chip.className = "chip chip-ok";
  } else if (deviceState === "NO_TARGET") {
    chip.textContent = "No Target";
    chip.className = "chip";
  } else if (deviceState) {
    chip.textContent = "Sensing";
    chip.className = "chip chip-active";
  } else {
    chip.textContent = "Waiting";
    chip.className = "chip";
  }

  // Person state
  const stateEl = document.getElementById("s-device-state");
  if (stateEl) stateEl.textContent = STATE_LABELS[deviceState] || deviceState || "--";

  // Target count
  const countEl = document.getElementById("s-target-count");
  if (countEl) {
    const count = data.target_count;
    countEl.textContent = (count != null) ? String(count) : "--";
  }

  // Truncation warning
  const truncEl = document.getElementById("s-truncated");
  if (truncEl) show(truncEl, !!data.targets_truncated);

  // Heart rate
  const hrEl = document.getElementById("s-hr");
  if (hrEl) {
    const hr = data.heart_rate_bpm;
    hrEl.textContent = (hr != null) ? hr.toFixed(1) : "--";
  }

  // Breathing rate
  const brEl = document.getElementById("s-br");
  if (brEl) {
    const br = data.breath_rate_bpm;
    brEl.textContent = (br != null) ? br.toFixed(1) : "--";
  }

  // Lux
  const luxEl = document.getElementById("s-lux");
  if (luxEl) {
    const lux = data.lux;
    luxEl.textContent = (lux != null) ? lux.toFixed(0) : "--";
  }

  // Mode / last updated
  const modeEl = document.getElementById("s-mode");
  if (modeEl) modeEl.textContent = data.mode || "--";

  const tsEl = document.getElementById("s-updated");
  if (tsEl) tsEl.textContent = formatAge(data.updated_at);
}

async function pollSensor() {
  try {
    const resp = await fetchWithTimeout("/sensor", {}, 2000);
    if (resp.ok) {
      const data = await resp.json();
      updateSensorUI(data);
    }
  } catch (e) {
    // silently ignore fetch errors
  }
}

function startSensorPolling() {
  pollSensor();
  setInterval(pollSensor, SENSOR_POLL_MS);
}

/* ---- Init ---- */

async function init() {
  const loading = document.getElementById("loading");
  const statusEl = document.getElementById("status");
  const formPanel = document.getElementById("form-panel");
  const configuredPanel = document.getElementById("configured");
  const saveBtn = document.getElementById("save-btn");
  const changeKeyBtn = document.getElementById("change-key-btn");
  const input = document.getElementById("api-key");

  show(loading, true);
  show(formPanel, false);
  show(configuredPanel, false);

  const st = (await waitForStatus()) || { has_key: false };

  if (st.has_key) {
    show(configuredPanel, true);
  } else {
    show(formPanel, true);
  }
  show(loading, false);

  // Start sensor dashboard polling
  startSensorPolling();

  changeKeyBtn.addEventListener("click", () => {
    show(configuredPanel, false);
    show(formPanel, true);
    input.value = "";
    statusEl.textContent = "";
    statusEl.className = "status";
  });

  input.addEventListener("input", () => {
    input.classList.remove("error");
  });

  saveBtn.addEventListener("click", async () => {
    const key = input.value.trim();
    if (!key) {
      statusEl.textContent = "Please enter a valid key.";
      statusEl.className = "status warn";
      input.classList.add("error");
      return;
    }
    statusEl.textContent = "Validating API key...";
    statusEl.className = "status";
    input.classList.remove("error");
    try {
      const validation = await validateKey(key);
      if (!validation.valid) {
        statusEl.textContent = "Invalid API key. Please check your key and try again.";
        statusEl.className = "status error";
        input.classList.add("error");
        return;
      }
      statusEl.textContent = "Key valid! Saving...";
      statusEl.className = "status ok";
      await saveKey(key);
      statusEl.textContent = "Saved. Reloading\u2026";
      statusEl.className = "status ok";
      window.location.reload();
    } catch (e) {
      input.classList.add("error");
      if (e.message === "invalid_api_key") {
        statusEl.textContent = "Invalid API key. Please check your key and try again.";
      } else {
        statusEl.textContent = "Failed to validate/save key. Please try again.";
      }
      statusEl.className = "status error";
    }
  });
}

window.addEventListener("DOMContentLoaded", init);
