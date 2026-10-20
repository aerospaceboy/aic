/** Click-scoped stopwatch, shared by CoStudio's isolated panel contexts. */
export type RoundClockRecord = {
  version: 2;
  taskId: string;
  missionTaskId: string;
  startedAt: number;
  stoppedAt?: number;
  phase: "pending" | "running" | "completed" | "failed" | "canceled";
};

// Never import the old unbounded start-only anchor into a new stopwatch.
const STORAGE_KEY = "yzbot.competition.roundClock.v2";
let memoryRecord: RoundClockRecord | undefined;
const listeners = new Set<() => void>();

function safeStorage(): Storage | undefined {
  try { return window.localStorage; } catch { return undefined; }
}

function persist(record: RoundClockRecord | undefined): void {
  memoryRecord = record;
  try {
    const storage = safeStorage();
    if (record) {
      storage?.setItem(STORAGE_KEY, JSON.stringify(record));
    } else {
      storage?.removeItem(STORAGE_KEY);
    }
  } catch { /* blocked storage: retain this context's clock */ }
  for (const listener of listeners) { listener(); }
}

export function readRoundClock(): RoundClockRecord | undefined {
  try {
    const storage = safeStorage();
    if (!storage) { return memoryRecord; }
    const raw = storage.getItem(STORAGE_KEY);
    if (!raw) { return undefined; }
    let record: RoundClockRecord;
    try { record = JSON.parse(raw) as RoundClockRecord; } catch { return undefined; }
    if (!record || record.version !== 2 || typeof record.taskId !== "string" || !record.taskId ||
        typeof record.missionTaskId !== "string" ||
        !Number.isFinite(record.startedAt) || record.startedAt <= 0 ||
        !["pending", "running", "completed", "failed", "canceled"].includes(record.phase) ||
        (record.stoppedAt !== undefined &&
          (!Number.isFinite(record.stoppedAt) || record.stoppedAt < record.startedAt)) ||
        (["completed", "failed", "canceled"].includes(record.phase) !==
          (record.stoppedAt !== undefined))) {
      return undefined;
    }
    return record;
  } catch { return memoryRecord; }
}

/** The sole start path: a valid operator click, before sending any chunks. */
export function markRoundStart(submissionId: string, when: number = Date.now()): void {
  persist({ version: 2, taskId: submissionId, missionTaskId: "",
    startedAt: when, phase: "pending" });
}

export function clearRoundStart(): void { persist(undefined); }

export function finishRoundSubmission(submissionId: string, when: number = Date.now()): void {
  const record = readRoundClock();
  if (record?.taskId === submissionId && record.stoppedAt === undefined) {
    persist({ ...record, phase: "failed", stoppedAt: Math.max(record.startedAt, when) });
  }
}

function object(value: unknown): Record<string, unknown> {
  return value !== null && typeof value === "object"
    ? value as Record<string, unknown> : {};
}

/** Pure transition: idle, stale/unrelated telemetry and duplicate finals are inert. */
export function advanceRoundClock(
  record: RoundClockRecord | undefined,
  message: unknown,
  now: number = Date.now(),
): RoundClockRecord | undefined {
  if (!record || record.stoppedAt !== undefined) { return record; }
  let status = object(message);
  if (typeof status["data"] === "string") {
    try { status = object(JSON.parse(status["data"])); } catch { return record; }
  }
  const evidence = object(status["parse_result"]);
  let next = record;
  if (evidence["request_id"] === record.taskId) {
    if (evidence["state"] === "failed") {
      return { ...record, phase: "failed", stoppedAt: Math.max(record.startedAt, now) };
    }
    if (evidence["state"] === "published" && evidence["published"] === true &&
        typeof evidence["task_id"] === "string" && evidence["task_id"]) {
      next = { ...record, missionTaskId: evidence["task_id"] };
    }
  }
  // task_id in the legacy envelope is the parser's plan, not the executing
  // mission. Use the explicit mission identity; never substring-match IDs.
  const missionId = String(status["mission_task_id"] ?? "");
  if (!missionId || missionId !== (next.missionTaskId || next.taskId)) { return next; }
  const state = String(status["task_manager_state"] ?? status["current_state"] ?? "").toLowerCase();
  if (["completed", "failed", "canceled", "aborted"].includes(state)) {
    const finishedAt = status["mission_finished_at_unix_ms"];
    // A persisted backend endpoint prevents disconnection/reload latency from
    // being charged to the operator. Missing endpoints cannot fabricate one.
    if (typeof finishedAt !== "number" || !Number.isFinite(finishedAt) ||
        finishedAt < next.startedAt || finishedAt > now + 1000) { return next; }
    return { ...next, stoppedAt: finishedAt,
      phase: state === "completed" ? "completed" : state === "canceled" ? "canceled" : "failed" };
  }
  return state && state !== "idle" ? { ...next, phase: "running" } : next;
}

/** Call only for real status frames, never demo/fallback state. */
export function observeRoundStatus(message: unknown): void {
  const current = readRoundClock();
  const next = advanceRoundClock(current, message);
  if (JSON.stringify(next) !== JSON.stringify(current)) { persist(next); }
}

export function subscribeRoundClock(listener: () => void): () => void {
  listeners.add(listener);
  const onStorage = (event: StorageEvent): void => {
    if (event.key === STORAGE_KEY || event.key === null) { listener(); }
  };
  window.addEventListener("storage", onStorage);
  return () => {
    listeners.delete(listener);
    window.removeEventListener("storage", onStorage);
  };
}
