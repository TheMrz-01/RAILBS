import { mkdir, readdir, readFile, stat, writeFile } from "node:fs/promises";
import { createWriteStream, existsSync } from "node:fs";
import { join, normalize } from "node:path";

type RobotStatus = {
  timeMs: number;
  state: string;
  frontMarker: number;
  rearMarker: number;
  distanceCm: number;
  leftPwm: number;
  rightPwm: number;
  tunnelDetected: boolean;
  tunnelConfirmed: boolean;
  estimatedSpeedMps: number;
  missionStarted: boolean;
  telemetryLogCount: number;
  eventLogCount: number;
  ip: string;
};

type AppState = {
  esp32BaseUrl: string;
  pollMs: number;
  polling: boolean;
  connected: boolean;
  lastError: string | null;
  lastStatus: RobotStatus | null;
  samples: RobotStatus[];
  activeRun: RunState | null;
  completedRunIds: string[];
};

type RunState = {
  id: string;
  dir: string;
  telemetryPath: string;
  eventsPath: string;
  summaryPath: string;
  telemetryStream: ReturnType<typeof createWriteStream>;
  eventsStream: ReturnType<typeof createWriteStream>;
  startedAt: string;
  sampleCount: number;
  lastEventCount: number;
};

const rootDir = process.cwd();
const publicDir = join(rootDir, "public");
const runsDir = join(rootDir, "runs");
const port = Number(Bun.env.PORT ?? 3000);

const state: AppState = {
  esp32BaseUrl: Bun.env.ESP32_URL ?? "http://192.168.4.1",
  pollMs: Number(Bun.env.POLL_MS ?? 100),
  polling: true,
  connected: false,
  lastError: null,
  lastStatus: null,
  samples: [],
  activeRun: null,
  completedRunIds: []
};

await mkdir(runsDir, { recursive: true });

function json(data: unknown, init: ResponseInit = {}) {
  return new Response(JSON.stringify(data), {
    ...init,
    headers: {
      "content-type": "application/json; charset=utf-8",
      "cache-control": "no-store",
      ...(init.headers ?? {})
    }
  });
}

function text(data: string, init: ResponseInit = {}) {
  return new Response(data, {
    ...init,
    headers: {
      "content-type": "text/plain; charset=utf-8",
      "cache-control": "no-store",
      ...(init.headers ?? {})
    }
  });
}

function sanitizeBaseUrl(value: string) {
  const trimmed = value.trim().replace(/\/+$/, "");
  if (!trimmed.startsWith("http://") && !trimmed.startsWith("https://")) {
    throw new Error("ESP32 URL must start with http:// or https://");
  }
  return trimmed;
}

function csvEscape(value: unknown) {
  const textValue = String(value ?? "");
  if (!/[",\n]/.test(textValue)) return textValue;
  return `"${textValue.replaceAll('"', '""')}"`;
}

function statusToCsv(status: RobotStatus) {
  return [
    status.timeMs,
    status.state,
    status.frontMarker,
    status.rearMarker,
    status.distanceCm,
    status.leftPwm,
    status.rightPwm,
    status.tunnelDetected ? 1 : 0,
    status.tunnelConfirmed ? 1 : 0,
    status.estimatedSpeedMps,
    status.missionStarted ? 1 : 0
  ].map(csvEscape).join(",") + "\n";
}

function currentRunId() {
  const date = new Date();
  const stamp = date.toISOString().replaceAll(":", "-").replace(/\.\d+Z$/, "Z");
  return `run-${stamp}`;
}

async function startRun(status: RobotStatus) {
  if (state.activeRun) return;

  const id = currentRunId();
  const dir = join(runsDir, id);
  await mkdir(dir, { recursive: true });

  const telemetryPath = join(dir, "telemetry.csv");
  const eventsPath = join(dir, "events.csv");
  const summaryPath = join(dir, "summary.json");
  const telemetryStream = createWriteStream(telemetryPath, { flags: "a" });
  const eventsStream = createWriteStream(eventsPath, { flags: "a" });

  telemetryStream.write("timeMs,state,frontMarker,rearMarker,distanceCm,leftPwm,rightPwm,tunnelDetected,tunnelConfirmed,estimatedSpeedMps,missionStarted\n");
  eventsStream.write("timeMs,event,value\n");

  state.activeRun = {
    id,
    dir,
    telemetryPath,
    eventsPath,
    summaryPath,
    telemetryStream,
    eventsStream,
    startedAt: new Date().toISOString(),
    sampleCount: 0,
    lastEventCount: status.eventLogCount ?? 0
  };

  writeStatusSample(status);
  await syncEventsFromEsp32();
}

async function stopRun(reason: string) {
  const run = state.activeRun;
  if (!run) return;

  await syncEventsFromEsp32();
  run.telemetryStream.end();
  run.eventsStream.end();

  const last = state.lastStatus;
  const summary = {
    id: run.id,
    startedAt: run.startedAt,
    endedAt: new Date().toISOString(),
    reason,
    sampleCount: run.sampleCount,
    finalState: last?.state ?? null,
    finalFrontMarker: last?.frontMarker ?? null,
    finalRearMarker: last?.rearMarker ?? null,
    finalTimeMs: last?.timeMs ?? null,
    maxEstimatedSpeedMps: maxOf(state.samples, "estimatedSpeedMps"),
    minDistanceCm: minOf(state.samples.filter((s) => s.distanceCm < 900), "distanceCm")
  };

  await writeFile(run.summaryPath, JSON.stringify(summary, null, 2));
  state.completedRunIds.unshift(run.id);
  state.activeRun = null;
}

function maxOf(samples: RobotStatus[], key: keyof RobotStatus) {
  const values = samples.map((sample) => Number(sample[key])).filter(Number.isFinite);
  return values.length ? Math.max(...values) : null;
}

function minOf(samples: RobotStatus[], key: keyof RobotStatus) {
  const values = samples.map((sample) => Number(sample[key])).filter(Number.isFinite);
  return values.length ? Math.min(...values) : null;
}

function writeStatusSample(status: RobotStatus) {
  const run = state.activeRun;
  if (!run) return;
  run.telemetryStream.write(statusToCsv(status));
  run.sampleCount++;
}

async function fetchEsp32Text(path: string) {
  const response = await fetch(`${state.esp32BaseUrl}${path}`, {
    signal: AbortSignal.timeout(Math.max(1200, state.pollMs * 4))
  });
  if (!response.ok) {
    throw new Error(`${path} returned ${response.status}`);
  }
  return response.text();
}

async function fetchEsp32Status() {
  const response = await fetch(`${state.esp32BaseUrl}/status`, {
    signal: AbortSignal.timeout(Math.max(1200, state.pollMs * 4))
  });
  if (!response.ok) {
    throw new Error(`/status returned ${response.status}`);
  }
  return response.json() as Promise<RobotStatus>;
}

async function syncEventsFromEsp32() {
  const run = state.activeRun;
  if (!run) return;

  const csv = await fetchEsp32Text("/events");
  const lines = csv.trim().split("\n").slice(1);
  if (lines.length <= run.lastEventCount) return;

  for (const line of lines.slice(run.lastEventCount)) {
    run.eventsStream.write(`${line}\n`);
  }

  run.lastEventCount = lines.length;
}

async function pollOnce() {
  if (!state.polling) return;

  try {
    const previous = state.lastStatus;
    const status = await fetchEsp32Status();

    state.connected = true;
    state.lastError = null;
    state.lastStatus = status;
    state.samples.push(status);
    if (state.samples.length > 600) state.samples.shift();

    if (status.missionStarted && !state.activeRun) {
      await startRun(status);
    } else if (state.activeRun) {
      writeStatusSample(status);
      if ((status.eventLogCount ?? 0) !== state.activeRun.lastEventCount) {
        await syncEventsFromEsp32();
      }
    }

    const missionEnded = previous?.missionStarted && !status.missionStarted;
    const terminalState = status.state === "Finished" || status.state === "EmergencyStop";
    if (state.activeRun && (missionEnded || terminalState)) {
      await stopRun(status.state);
    }
  } catch (error) {
    state.connected = false;
    state.lastError = error instanceof Error ? error.message : String(error);
  }
}

setInterval(() => {
  pollOnce().catch((error) => {
    state.connected = false;
    state.lastError = error instanceof Error ? error.message : String(error);
  });
}, state.pollMs);

function safeRunPath(runId: string, file: string) {
  if (!/^run-[A-Za-z0-9_.:-]+$/.test(runId)) return null;
  const path = normalize(join(runsDir, runId, file));
  if (!path.startsWith(runsDir)) return null;
  return path;
}

async function listRuns() {
  const entries = await readdir(runsDir, { withFileTypes: true });
  const runs = await Promise.all(entries
    .filter((entry) => entry.isDirectory() && entry.name.startsWith("run-"))
    .map(async (entry) => {
      const summaryPath = join(runsDir, entry.name, "summary.json");
      let summary = null;
      try {
        summary = JSON.parse(await readFile(summaryPath, "utf8"));
      } catch {
        summary = null;
      }
      return { id: entry.name, summary };
    }));
  return runs.sort((a, b) => b.id.localeCompare(a.id));
}

async function serveStatic(pathname: string) {
  const filePath = pathname === "/" ? join(publicDir, "index.html") : normalize(join(publicDir, pathname));
  if (!filePath.startsWith(publicDir)) return new Response("Not found", { status: 404 });

  try {
    const info = await stat(filePath);
    if (!info.isFile()) return new Response("Not found", { status: 404 });
    const file = Bun.file(filePath);
    return new Response(file, {
      headers: { "cache-control": "no-store" }
    });
  } catch {
    return new Response("Not found", { status: 404 });
  }
}

const server = Bun.serve({
  port,
  async fetch(request) {
    const url = new URL(request.url);
    const pathname = url.pathname;

    if (pathname === "/api/state") {
      return json({
        esp32BaseUrl: state.esp32BaseUrl,
        pollMs: state.pollMs,
        polling: state.polling,
        connected: state.connected,
        lastError: state.lastError,
        lastStatus: state.lastStatus,
        activeRunId: state.activeRun?.id ?? null,
        recentSamples: state.samples,
        completedRunIds: state.completedRunIds
      });
    }

    if (pathname === "/api/config" && request.method === "POST") {
      const body = await request.json().catch(() => null) as { esp32BaseUrl?: string; pollMs?: number; polling?: boolean } | null;
      if (!body) return json({ error: "Invalid JSON" }, { status: 400 });

      if (typeof body.esp32BaseUrl === "string") {
        try {
          state.esp32BaseUrl = sanitizeBaseUrl(body.esp32BaseUrl);
        } catch (error) {
          return json({ error: error instanceof Error ? error.message : String(error) }, { status: 400 });
        }
      }
      if (typeof body.pollMs === "number") {
        state.pollMs = Math.max(50, Math.min(2000, Math.round(body.pollMs)));
      }
      if (typeof body.polling === "boolean") {
        state.polling = body.polling;
      }
      return json({ ok: true, esp32BaseUrl: state.esp32BaseUrl, pollMs: state.pollMs, polling: state.polling });
    }

    if (pathname === "/api/esp32/start" && request.method === "POST") {
      return text(await fetchEsp32Text("/start"));
    }

    if (pathname === "/api/esp32/stop" && request.method === "POST") {
      return text(await fetchEsp32Text("/stop"));
    }

    if (pathname === "/api/esp32/reset" && request.method === "POST") {
      return text(await fetchEsp32Text("/reset"));
    }

    if (pathname === "/api/esp32/clear-log" && request.method === "POST") {
      return text(await fetchEsp32Text("/log/clear"));
    }

    if (pathname === "/api/runs") {
      return json(await listRuns());
    }

    const runFileMatch = pathname.match(/^\/api\/runs\/([^/]+)\/(telemetry\.csv|events\.csv|summary\.json)$/);
    if (runFileMatch) {
      const [, runId, file] = runFileMatch;
      const path = safeRunPath(runId, file);
      if (!path || !existsSync(path)) return new Response("Not found", { status: 404 });
      const contentType = file.endsWith(".json") ? "application/json; charset=utf-8" : "text/csv; charset=utf-8";
      return new Response(Bun.file(path), {
        headers: {
          "content-type": contentType,
          "content-disposition": `attachment; filename=${file}`
        }
      });
    }

    return serveStatic(pathname);
  }
});

console.log(`RailBot dashboard: http://localhost:${server.port}`);
console.log(`ESP32 source: ${state.esp32BaseUrl}`);
