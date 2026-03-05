/* Dashboard client — WebSocket, vitals card, radar canvas, history chart. */

(function () {
  "use strict";

  // ---- Constants ----

  const STATE_LABELS = {
    NO_TARGET: "No one detected",
    MULTI_TARGET: "Multiple people",
    PRESENT_FAR: "Someone nearby (far)",
    MOVING: "Person moving",
    STILL_NEAR: "Person still (close)",
    RESTING_VITALS: "Resting \u2014 vitals active",
  };

  const STATE_BADGE_CLASS = {
    RESTING_VITALS: "state-resting",
    STILL_NEAR: "state-resting",
    MOVING: "state-active",
    PRESENT_FAR: "state-active",
    MULTI_TARGET: "state-warn",
    NO_TARGET: "state-none",
  };

  const STALE_THRESHOLD_MS = 30000;
  const STALENESS_CHECK_MS = 5000;
  const MAX_CHART_POINTS = 500;
  const RECONNECT_MIN_MS = 1000;
  const RECONNECT_MAX_MS = 30000;

  // Radar constants
  const RADAR_W = 300;
  const RADAR_H = 300;
  const RADAR_BG = "#0a0f1e";
  const RADAR_RING_COLOR = "rgba(69, 196, 255, 0.15)";
  const RADAR_LABEL_COLOR = "rgba(159, 182, 215, 0.6)";
  const RADAR_MAX_RANGE = 2.0; // meters
  const RADAR_RINGS = [0.5, 1.0, 1.5]; // meters

  const TARGET_COLORS = {
    RESTING_VITALS: "#5ef0c1",
    STILL_NEAR: "#5ef0c1",
    MOVING: "#45c4ff",
    PRESENT_FAR: "#45c4ff",
  };
  const TARGET_COLOR_DEFAULT = "#6b7280";
  const TARGET_RADIUS = 6;

  // ---- State ----

  let lastUpdate = 0;
  let reconnectDelay = RECONNECT_MIN_MS;
  let wsConnected = false;
  let vitalsChart = null;

  // DOM element refs (populated on init)
  let elHrValue = null;
  let elBrValue = null;
  let elStateBadge = null;
  let elTargetCount = null;
  let elLuxValue = null;
  let elStatusDot = null;
  let elStatusText = null;
  let radarCanvas = null;
  let radarCtx = null;

  // ---- A. WebSocket Connection ----

  function connectWS() {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const url = `${proto}//${location.host}/ws/sensor`;
    let ws;
    try {
      ws = new WebSocket(url);
    } catch (e) {
      scheduleReconnect();
      return;
    }

    ws.onopen = () => {
      wsConnected = true;
      reconnectDelay = RECONNECT_MIN_MS;
      updateConnectionStatus();
    };

    ws.onmessage = (e) => {
      let data;
      try {
        data = JSON.parse(e.data);
      } catch (err) {
        return;
      }
      lastUpdate = Date.now();
      updateVitals(data);
      updateRadar(data);
      appendToChart(data);
      updateConnectionStatus();
    };

    ws.onclose = () => {
      wsConnected = false;
      updateConnectionStatus();
      scheduleReconnect();
    };

    ws.onerror = () => {
      // onclose will fire after onerror
    };
  }

  function scheduleReconnect() {
    setTimeout(connectWS, reconnectDelay);
    reconnectDelay = Math.min(reconnectDelay * 2, RECONNECT_MAX_MS);
  }

  // ---- B. Vitals Card Updater ----

  function updateVitals(data) {
    if (!elHrValue) return;

    // Heart rate
    const hr = data.heart_rate_bpm;
    if (hr != null) {
      elHrValue.textContent = hr.toFixed(0);
      elHrValue.classList.remove("vitals-inactive");
      elHrValue.classList.add("vitals-hr");
    } else {
      elHrValue.textContent = "--";
      elHrValue.classList.add("vitals-inactive");
      elHrValue.classList.remove("vitals-hr");
    }

    // Breathing rate
    const br = data.breath_rate_bpm;
    if (br != null) {
      elBrValue.textContent = br.toFixed(0);
      elBrValue.classList.remove("vitals-inactive");
      elBrValue.classList.add("vitals-br");
    } else {
      elBrValue.textContent = "--";
      elBrValue.classList.add("vitals-inactive");
      elBrValue.classList.remove("vitals-br");
    }

    // Device state badge
    const state = data.device_state || null;
    const label = state ? (STATE_LABELS[state] || state) : "--";
    elStateBadge.textContent = label;
    elStateBadge.className = "state-badge " + (STATE_BADGE_CLASS[state] || "state-none");

    // Target count
    const tc = data.target_count;
    if (tc != null) {
      let text = `Targets: ${tc}`;
      if (data.targets_truncated) {
        text += " (truncated)";
      }
      elTargetCount.textContent = text;
    } else {
      elTargetCount.textContent = "Targets: --";
    }

    // Lux / proximity
    const lux = data.lux;
    if (lux != null) {
      let proximity;
      if (lux < 40) {
        proximity = "Close presence";
      } else if (lux < 120) {
        proximity = "Partial occlusion";
      } else {
        proximity = "Clear path";
      }
      elLuxValue.textContent = `Lux: ${lux.toFixed(0)} (${proximity})`;
    } else {
      elLuxValue.textContent = "Lux: --";
    }
  }

  function updateConnectionStatus() {
    if (!elStatusDot) return;

    if (!wsConnected) {
      elStatusDot.className = "status-dot dot-red";
      elStatusText.textContent = "Disconnected";
      return;
    }

    const age = Date.now() - lastUpdate;
    if (lastUpdate === 0 || age > STALE_THRESHOLD_MS) {
      elStatusDot.className = "status-dot dot-orange";
      elStatusText.textContent = lastUpdate === 0 ? "Waiting for data..." : "Stale data";
    } else {
      elStatusDot.className = "status-dot dot-green";
      elStatusText.textContent = "Live";
    }
  }

  // ---- C. Radar Canvas ----

  function drawRadarBackground() {
    if (!radarCtx) return;
    const cx = RADAR_W / 2;
    const cy = RADAR_H - 20; // sensor at center-bottom with small margin
    const maxPx = RADAR_H - 40; // max radius in pixels for RADAR_MAX_RANGE meters

    radarCtx.fillStyle = RADAR_BG;
    radarCtx.fillRect(0, 0, RADAR_W, RADAR_H);

    // Range rings (180-degree arcs above sensor)
    radarCtx.setLineDash([4, 4]);
    radarCtx.strokeStyle = RADAR_RING_COLOR;
    radarCtx.lineWidth = 1;
    radarCtx.font = "10px sans-serif";
    radarCtx.fillStyle = RADAR_LABEL_COLOR;
    radarCtx.textAlign = "center";

    for (const ring of RADAR_RINGS) {
      const rPx = (ring / RADAR_MAX_RANGE) * maxPx;
      radarCtx.beginPath();
      radarCtx.arc(cx, cy, rPx, Math.PI, 0); // 180-degree arc (top half)
      radarCtx.stroke();
      // Label at the top of each ring
      radarCtx.fillText(`${ring}m`, cx, cy - rPx - 4);
    }

    radarCtx.setLineDash([]);

    // Sensor marker at bottom-center
    radarCtx.fillStyle = "rgba(255, 255, 255, 0.3)";
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, 3, 0, Math.PI * 2);
    radarCtx.fill();
  }

  function updateRadar(data) {
    if (!radarCtx) return;

    drawRadarBackground();

    const targets = data.targets;
    if (!Array.isArray(targets) || targets.length === 0) return;

    const cx = RADAR_W / 2;
    const cy = RADAR_H - 20;
    const maxPx = RADAR_H - 40;
    const deviceState = data.device_state || "";

    for (const t of targets) {
      // targets have x (meters, + right) and y (meters, + forward)
      const tx = t.x;
      const ty = t.y;
      if (tx == null || ty == null) continue;

      // Convert meters to canvas pixels
      const pxX = cx + (tx / RADAR_MAX_RANGE) * maxPx;
      const pxY = cy - (ty / RADAR_MAX_RANGE) * maxPx;

      // Clamp to canvas bounds
      const drawX = Math.max(TARGET_RADIUS, Math.min(RADAR_W - TARGET_RADIUS, pxX));
      const drawY = Math.max(TARGET_RADIUS, Math.min(RADAR_H - TARGET_RADIUS, pxY));

      // Choose color based on device state
      const color = TARGET_COLORS[deviceState] || TARGET_COLOR_DEFAULT;

      // Glow effect
      radarCtx.shadowColor = color;
      radarCtx.shadowBlur = 10;

      radarCtx.fillStyle = color;
      radarCtx.beginPath();
      radarCtx.arc(drawX, drawY, TARGET_RADIUS, 0, Math.PI * 2);
      radarCtx.fill();

      radarCtx.shadowBlur = 0;
    }
  }

  // ---- D. History Chart (Chart.js) ----

  function initChart() {
    const canvas = document.getElementById("vitals-chart");
    if (!canvas || typeof Chart === "undefined") return;

    vitalsChart = new Chart(canvas, {
      type: "line",
      data: {
        datasets: [
          {
            label: "Heart Rate",
            data: [],
            borderColor: "#ff6b6b",
            backgroundColor: "rgba(255, 107, 107, 0.1)",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.3,
            yAxisID: "yHR",
          },
          {
            label: "Breathing Rate",
            data: [],
            borderColor: "#45c4ff",
            backgroundColor: "rgba(69, 196, 255, 0.1)",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.3,
            yAxisID: "yBR",
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        interaction: { mode: "index", intersect: false },
        plugins: {
          legend: {
            labels: { color: "rgba(234, 242, 255, 0.7)", boxWidth: 12, padding: 16 },
          },
          tooltip: {
            backgroundColor: "rgba(11, 18, 36, 0.9)",
            titleColor: "#eaf2ff",
            bodyColor: "#9fb6d7",
            borderColor: "rgba(255, 255, 255, 0.08)",
            borderWidth: 1,
          },
        },
        scales: {
          x: {
            type: "time",
            time: { tooltipFormat: "HH:mm:ss", displayFormats: { minute: "HH:mm", hour: "HH:mm" } },
            ticks: { color: "rgba(159, 182, 215, 0.5)", maxTicksLimit: 8 },
            grid: { color: "rgba(255, 255, 255, 0.04)" },
          },
          yHR: {
            type: "linear",
            position: "left",
            min: 40,
            max: 140,
            title: { display: true, text: "HR (bpm)", color: "rgba(255, 107, 107, 0.7)" },
            ticks: { color: "rgba(255, 107, 107, 0.5)" },
            grid: { color: "rgba(255, 255, 255, 0.04)" },
          },
          yBR: {
            type: "linear",
            position: "right",
            min: 4,
            max: 30,
            title: { display: true, text: "BR (bpm)", color: "rgba(69, 196, 255, 0.7)" },
            ticks: { color: "rgba(69, 196, 255, 0.5)" },
            grid: { drawOnChartArea: false },
          },
        },
      },
    });
  }

  function appendToChart(data) {
    if (!vitalsChart) return;

    const hr = data.heart_rate_bpm;
    const br = data.breath_rate_bpm;
    if (hr == null && br == null) return;

    const ts = data.updated_at ? new Date(data.updated_at * 1000) : new Date();
    const hrDataset = vitalsChart.data.datasets[0].data;
    const brDataset = vitalsChart.data.datasets[1].data;

    if (hr != null) {
      hrDataset.push({ x: ts, y: hr });
      if (hrDataset.length > MAX_CHART_POINTS) hrDataset.shift();
    }

    if (br != null) {
      brDataset.push({ x: ts, y: br });
      if (brDataset.length > MAX_CHART_POINTS) brDataset.shift();
    }

    vitalsChart.update("none"); // skip animation for incremental updates
  }

  function loadHistory() {
    if (!vitalsChart) return;

    fetch("/api/vitals/history?hours=4")
      .then((resp) => {
        if (!resp.ok) throw new Error("fetch failed");
        return resp.json();
      })
      .then((json) => {
        const rows = json.rows || [];
        const hrData = [];
        const brData = [];

        for (const row of rows) {
          const ts = new Date(row.timestamp);
          if (row.heart_rate_bpm != null) {
            hrData.push({ x: ts, y: row.heart_rate_bpm });
          }
          if (row.breath_rate_bpm != null) {
            brData.push({ x: ts, y: row.breath_rate_bpm });
          }
        }

        // Keep only the latest MAX_CHART_POINTS
        vitalsChart.data.datasets[0].data = hrData.slice(-MAX_CHART_POINTS);
        vitalsChart.data.datasets[1].data = brData.slice(-MAX_CHART_POINTS);
        vitalsChart.update();
      })
      .catch(() => {
        // History not available yet, chart starts empty
      });
  }

  // ---- E. Initialization ----

  function init() {
    // Grab DOM refs
    elHrValue = document.getElementById("hr-value");
    elBrValue = document.getElementById("br-value");
    elStateBadge = document.getElementById("state-badge");
    elTargetCount = document.getElementById("target-count");
    elLuxValue = document.getElementById("lux-value");
    elStatusDot = document.getElementById("status-dot");
    elStatusText = document.getElementById("status-text");
    radarCanvas = document.getElementById("radar-canvas");

    if (radarCanvas) {
      radarCtx = radarCanvas.getContext("2d");
      drawRadarBackground();
    }

    // Init chart (if Chart.js is loaded)
    initChart();
    loadHistory();

    // Connect WebSocket
    connectWS();

    // Staleness checker
    setInterval(updateConnectionStatus, STALENESS_CHECK_MS);
  }

  window.addEventListener("DOMContentLoaded", init);
})();
