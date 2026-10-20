/** Helpers shared by the competition panels. */

export type JsonRecord = Record<string, unknown>;

export function asRecord(value: unknown): JsonRecord {
  if (value !== null && typeof value === "object" && !Array.isArray(value)) {
    return value as JsonRecord;
  }
  return {};
}

function valueAt(value: unknown, path: string): unknown {
  let current = value;
  for (const part of path.split(".")) {
    if (part.length === 0) {
      continue;
    }
    if (Array.isArray(current)) {
      const index = Number.parseInt(part, 10);
      if (!Number.isFinite(index)) {
        return undefined;
      }
      current = current[index];
    } else if (current !== null && typeof current === "object") {
      current = (current as JsonRecord)[part];
    } else {
      return undefined;
    }
  }
  return current;
}

export function getPath(message: unknown, path: string | undefined): unknown {
  if (path === undefined || path.length === 0) {
    return undefined;
  }
  return valueAt(message, path);
}

export function formatPathValue(
  message: unknown,
  path: string | undefined,
  prefix: string | undefined,
  suffix: string | undefined,
  fallback: string,
): string {
  const value = getPath(message, path);
  if (value === undefined || value === null) {
    return fallback;
  }
  const text =
    typeof value === "object" ? JSON.stringify(value) : String(value);
  return `${prefix ?? ""}${text}${suffix ?? ""}`;
}

const ACTION_LABELS: Record<string, string> = {
  idle: "前往资源点",
  navigating_startup_clearance: "启动巡检",
  navigating_pickup: "前往资源点",
  waiting_perception: "识别物块",
  picking: "抓取物块",
  navigating_destination: "前往投放区",
  placing: "放置物块",
  verifying: "校验结果",
  releasing: "返回导航",
  completed: "任务完成",
  failed: "任务失败",
  canceled: "任务取消",
};

export function currentActionLabel(message: unknown, fallback: string): string {
  const record = asRecord(message);
  const state = String(record["current_state"] ?? record["task_manager_state"] ?? "");
  if (state === "navigating_destination") {
    const zone = String(record["current_target_zone"] ?? "").toUpperCase();
    if (zone === "A" || zone === "B" || zone === "C") {
      return `前往${zone}区`;
    }
  }
  return ACTION_LABELS[state] ?? fallback;
}

export function statusLabel(message: unknown, fallback: string): string {
  const record = asRecord(message);
  const currentState = String(record["current_state"] ?? "");
  const grasp = String(record["grasp_state"] ?? "").toLowerCase();
  const arm = String(record["arm_state"] ?? "").toLowerCase();
  if (currentState === "completed") {
    return "任务完成";
  }
  if (currentState === "failed") {
    return "任务失败";
  }
  if (grasp.includes("attach") || grasp.includes("grasp")) {
    return "夹持中";
  }
  if (grasp.includes("released") || arm.includes("released")) {
    return "已释放";
  }
  if (currentState === "picking") {
    return "抓取中";
  }
  if (currentState === "placing") {
    return "放置中";
  }
  if (currentState.startsWith("navigating")) {
    return "导航中";
  }
  if (currentState === "waiting_perception") {
    return "识别中";
  }
  return fallback;
}
