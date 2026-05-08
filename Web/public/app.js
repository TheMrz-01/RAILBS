const $ = (id) => document.getElementById(id);

const fields = {
  connection: $("connection-pill"),
  esp32Url: $("esp32-url"),
  pollMs: $("poll-ms"),
  saveConfig: $("save-config"),
  togglePolling: $("toggle-polling"),
  startMission: $("start-mission"),
  stopMission: $("stop-mission"),
  resetRobot: $("reset-robot"),
  clearEspLog: $("clear-esp-log"),
  state: $("state"),
  time: $("time"),
  frontMarker: $("front-marker"),
  rearMarker: $("rear-marker"),
  distance: $("distance"),
  speed: $("speed"),
  leftPwm: $("left-pwm"),
  rightPwm: $("right-pwm"),
  magnetSummary: $("magnet-summary"),
  magnetCount: $("magnet-count"),
  magnetPosition: $("magnet-position"),
  magnetProgressText: $("magnet-progress-text"),
  trackProgressFill: $("track-progress-fill"),
  activeRun: $("active-run"),
  lastError: $("last-error"),
  rawStatus: $("raw-status"),
  runs: $("runs"),
  refreshRuns: $("refresh-runs"),
  saveRobotConfig: $("save-robot-config"),
  robotConfig: {
    cruisePwm: $("cfg-cruisePwm"),
    approachPwm: $("cfg-approachPwm"),
    finishPwm: $("cfg-finishPwm"),
    brakePwm: $("cfg-brakePwm"),
    trim: $("cfg-trim"),
    markerSpacingCm: $("cfg-markerSpacingCm"),
    trackDistanceCm: $("cfg-trackDistanceCm"),
    tunnelApproachMarker: $("cfg-tunnelApproachMarker"),
    tunnelStopMarker: $("cfg-tunnelStopMarker"),
    tunnelExitMarker: $("cfg-tunnelExitMarker"),
    finishApproachMarker: $("cfg-finishApproachMarker"),
    finishMarker: $("cfg-finishMarker"),
    tunnelStopMs: $("cfg-tunnelStopMs"),
    tunnelDistanceCm: $("cfg-tunnelDistanceCm"),
    magnetDebounceUs: $("cfg-magnetDebounceUs")
  }
};

let latestState = null;

function formatMs(value) {
  if (!Number.isFinite(value)) return "-";
  return `${(value / 1000).toFixed(2)}s`;
}

function formatNumber(value, digits = 1, suffix = "") {
  if (!Number.isFinite(value)) return "-";
  return `${value.toFixed(digits)}${suffix}`;
}

async function api(path, options) {
  const response = await fetch(path, options);
  if (!response.ok) throw new Error(`${path} ${response.status} kodu döndürdü`);
  const contentType = response.headers.get("content-type") ?? "";
  if (contentType.includes("application/json")) return response.json();
  return response.text();
}

async function post(path, body = {}) {
  return api(path, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body)
  });
}

async function refreshState() {
  latestState = await api("/api/state");
  renderState(latestState);
}

function renderState(state) {
  const status = state.lastStatus;
  fields.esp32Url.value = state.esp32BaseUrl;
  fields.pollMs.value = state.pollMs;
  fields.togglePolling.textContent = state.polling ? "Sorgulamayı Durdur" : "Sorgulamayı Başlat";
  fields.connection.textContent = state.connected ? "Bağlandı" : "Bağlantı Yok";
  fields.connection.classList.toggle("ok", state.connected);
  fields.lastError.textContent = state.lastError ? `Son hata: ${state.lastError}` : "";
  fields.activeRun.textContent = state.activeRunId ? `Aktif: ${state.activeRunId}` : "Aktif sürüş yok";

  if (!status) {
    renderMagnetTracking(null, []);
    drawAllCharts([]);
    return;
  }

  fields.state.textContent = status.stateLabel ?? status.state;
  fields.time.textContent = formatMs(status.timeMs);
  fields.frontMarker.textContent = status.frontMarker;
  fields.rearMarker.textContent = status.rearMarker;
  fields.distance.textContent = formatNumber(status.distanceCm, 1, " cm");
  fields.speed.textContent = formatNumber(status.estimatedSpeedMps, 2, " m/s");
  fields.leftPwm.textContent = status.leftPwm;
  fields.rightPwm.textContent = status.rightPwm;
  fields.rawStatus.textContent = JSON.stringify(status, null, 2);
  fillRobotConfig(status);

  renderMagnetTracking(status, state.recentSamples ?? []);
  drawAllCharts(state.recentSamples ?? []);
}

function renderMagnetTracking(status, samples) {
  if (!status) {
    fields.magnetSummary.textContent = "Veri bekleniyor";
    fields.magnetCount.textContent = "-";
    fields.magnetPosition.textContent = "-";
    fields.magnetProgressText.textContent = "-";
    fields.trackProgressFill.style.width = "0%";
    drawTrackProgressChart([]);
    return;
  }

  const markerSpacingCm = Number(status.markerSpacingCm ?? 50);
  const trackDistanceCm = Number(status.trackDistanceCm ?? 2000);
  const currentMarker = Number(status.frontMarker ?? 0);
  const estimatedPositionCm = currentMarker * markerSpacingCm;
  const progressPercent = trackDistanceCm > 0 ? Math.min(100, Math.max(0, (estimatedPositionCm / trackDistanceCm) * 100)) : 0;
  const totalMarkers = markerSpacingCm > 0 ? Math.round(trackDistanceCm / markerSpacingCm) : 0;

  fields.magnetSummary.textContent = `${totalMarkers} markerlık parkurda canlı takip`;
  fields.magnetCount.textContent = `${currentMarker} / ${totalMarkers}`;
  fields.magnetPosition.textContent = `${(estimatedPositionCm / 100).toFixed(2)} m`;
  fields.magnetProgressText.textContent = `%${progressPercent.toFixed(1)}`;
  fields.trackProgressFill.style.width = `${progressPercent}%`;

  drawTrackProgressChart(samples);
}

function drawAllCharts(samples) {
  drawChart("marker-chart", samples, [
    { key: "frontMarker", label: "ön", color: "#5eead4" },
    { key: "rearMarker", label: "arka", color: "#fbbf24" }
  ]);
  drawChart("distance-chart", samples.filter((s) => s.distanceCm < 900), [
    { key: "distanceCm", label: "mesafe cm", color: "#93c5fd" }
  ]);
  drawChart("pwm-chart", samples, [
    { key: "leftPwm", label: "sol", color: "#a78bfa" },
    { key: "rightPwm", label: "sağ", color: "#fb7185" }
  ]);
  drawChart("speed-chart", samples, [
    { key: "estimatedSpeedMps", label: "hız", color: "#34d399" }
  ]);
}

function drawTrackProgressChart(samples) {
  const mapped = samples.map((sample) => {
    const markerSpacingCm = Number(sample.markerSpacingCm ?? 50);
    const trackDistanceCm = Number(sample.trackDistanceCm ?? 2000);
    const positionCm = Number(sample.frontMarker ?? 0) * markerSpacingCm;
    const progressPercent = trackDistanceCm > 0 ? Math.min(100, Math.max(0, (positionCm / trackDistanceCm) * 100)) : 0;
    return { ...sample, markerProgressPercent: progressPercent };
  });

  drawChart("magnet-progress-chart", mapped, [
    { key: "markerProgressPercent", label: "parkur %", color: "#fbbf24" }
  ]);
}

function fillRobotConfig(status) {
  for (const [key, input] of Object.entries(fields.robotConfig)) {
    if (!input || !(key in status)) continue;
    if (document.activeElement === input) continue;
    input.value = status[key];
  }
}

function collectRobotConfig() {
  const config = {};
  for (const [key, input] of Object.entries(fields.robotConfig)) {
    const value = Number(input.value);
    if (Number.isFinite(value)) config[key] = value;
  }
  return config;
}

function drawChart(canvasId, samples, series) {
  const canvas = $(canvasId);
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  const pad = { left: 48, right: 18, top: 18, bottom: 34 };

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0a101b";
  ctx.fillRect(0, 0, width, height);

  ctx.strokeStyle = "#1f2a40";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = pad.top + ((height - pad.top - pad.bottom) * i) / 4;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
  }

  if (!samples.length) {
    ctx.fillStyle = "#91a0b8";
    ctx.fillText("Veri bekleniyor", pad.left, height / 2);
    return;
  }

  const tMin = samples[0].timeMs;
  const tMax = samples.at(-1).timeMs || tMin + 1;
  const values = samples.flatMap((sample) => series.map((item) => Number(sample[item.key]))).filter(Number.isFinite);
  let yMin = Math.min(...values);
  let yMax = Math.max(...values);
  if (yMin === yMax) {
    yMin -= 1;
    yMax += 1;
  }
  const yPad = (yMax - yMin) * 0.08;
  yMin -= yPad;
  yMax += yPad;

  const xFor = (timeMs) => {
    const ratio = (timeMs - tMin) / Math.max(1, tMax - tMin);
    return pad.left + ratio * (width - pad.left - pad.right);
  };
  const yFor = (value) => {
    const ratio = (value - yMin) / Math.max(0.001, yMax - yMin);
    return height - pad.bottom - ratio * (height - pad.top - pad.bottom);
  };

  ctx.fillStyle = "#91a0b8";
  ctx.font = "12px system-ui";
  ctx.fillText(yMax.toFixed(1), 8, pad.top + 5);
  ctx.fillText(yMin.toFixed(1), 8, height - pad.bottom);
  ctx.fillText(`${((tMax - tMin) / 1000).toFixed(1)}s`, width - 58, height - 10);

  for (const item of series) {
    ctx.strokeStyle = item.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    samples.forEach((sample, index) => {
      const value = Number(sample[item.key]);
      if (!Number.isFinite(value)) return;
      const x = xFor(sample.timeMs);
      const y = yFor(value);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  let legendX = pad.left;
  for (const item of series) {
    ctx.fillStyle = item.color;
    ctx.fillRect(legendX, 8, 10, 10);
    ctx.fillStyle = "#d6e4ff";
    ctx.fillText(item.label, legendX + 15, 17);
    legendX += 80;
  }
}

async function refreshRuns() {
  const runs = await api("/api/runs");
  fields.runs.innerHTML = "";

  if (!runs.length) {
    fields.runs.textContent = "Henüz kayıtlı sürüş yok.";
    return;
  }

  for (const run of runs) {
    const row = document.createElement("div");
    row.className = "run-row";
    const summary = run.summary;
    row.innerHTML = `
      <strong>${run.id}</strong>
      <span>${summary ? `durum ${summary.finalState ?? "?"}, örnek ${summary.sampleCount ?? 0}` : "sürüş açık veya özet yok"}</span>
      <div class="run-links">
        <a href="/api/runs/${run.id}/telemetry.csv">telemetri.csv</a>
        <a href="/api/runs/${run.id}/events.csv">olaylar.csv</a>
        <a href="/api/runs/${run.id}/summary.json">özet.json</a>
      </div>
    `;
    fields.runs.append(row);
  }
}

fields.saveConfig.addEventListener("click", async () => {
  await post("/api/config", {
    esp32BaseUrl: fields.esp32Url.value,
    pollMs: Number(fields.pollMs.value)
  });
  await refreshState();
});

fields.togglePolling.addEventListener("click", async () => {
  await post("/api/config", { polling: !latestState?.polling });
  await refreshState();
});

fields.saveRobotConfig.addEventListener("click", async () => {
  await post("/api/esp32/config", collectRobotConfig());
  await refreshState();
});

fields.startMission.addEventListener("click", () => post("/api/esp32/start").then(refreshState).catch(showError));
fields.stopMission.addEventListener("click", () => post("/api/esp32/stop").then(refreshState).catch(showError));
fields.resetRobot.addEventListener("click", () => post("/api/esp32/reset").then(refreshState).catch(showError));
fields.clearEspLog.addEventListener("click", () => post("/api/esp32/clear-log").then(refreshState).catch(showError));
fields.refreshRuns.addEventListener("click", () => refreshRuns().catch(showError));

function showError(error) {
  fields.lastError.textContent = error instanceof Error ? error.message : String(error);
}

setInterval(() => {
  refreshState().catch(showError);
}, 250);

setInterval(() => {
  refreshRuns().catch(showError);
}, 3000);

await refreshState().catch(showError);
await refreshRuns().catch(showError);
