/** Unified competition display state shared by all custom panels. */

export type DataMode = "AUTO" | "DEMO" | "LIVE";

export const MODE_LABELS: Record<DataMode, string> = {
  AUTO: "自动模式",
  DEMO: "演示模式",
  LIVE: "实时模式",
};

export const MODE_DESCRIPTIONS: Record<DataMode, string> = {
  AUTO: "检测到 ROS2 实时数据时使用真实数据，未检测到实时数据时自动使用演示数据。",
  DEMO: "使用模拟数据演示完整比赛任务流程，不会向机器人、Nav2 或 MoveIt 发送真实控制指令。",
  LIVE: "使用 ROS2 实时数据。",
};

export interface TaskObjectState {
  category: string;
  count: number;
  targetRegion: string;
}

export interface TaskRuleState {
  condition: string;
  mapping: string;
}

export interface GraspRuleStepState {
  step: number;
  category: string;
  targetRegion: string;
  cubeName?: string;
}

export interface CompetitionState {
  mode: DataMode;
  question: string;
  redCount: number;
  blueCount: number;
  redDestination: "A" | "B" | "C" | "";
  blueDestination: "A" | "B" | "C" | "";
  taskObjects: TaskObjectState[];
  taskRules: TaskRuleState[];
  plannedGraspOrder: GraspRuleStepState[];
  actualGraspOrder: GraspRuleStepState[];
  currentGraspStep: number;
  currentRule?: GraspRuleStepState;
  graspRuleCompliance: string;
  selectionStrategy: string;
  selectionReason: string;
  selectionScore?: number;
  currentTaskIndex: number;
  totalTasks: number;
  currentAction: string;
  robotStatus: string;
  navigationStatus: string;
  armStatus: string;
  gripperStatus: string;
  completedCount: number;
  remainingCount: number;
  skippedCount: number;
  elapsedTime: number;
  currentTargetBlock: string;
  currentTargetZone: string;
  visibleTargets: string[];
  llmState: string;
  error: string;
  taskRunning: boolean;
  /** Round identifier published by the backend; changes once per round. */
  taskId: string;
  rawImageAvailable: boolean;
  detectionImageAvailable: boolean;
}

export const DEFAULT_DEMO_STATE: CompetitionState = {
  mode: "DEMO",
  taskId: "demo-round",
  question: "x代表红色，y代表蓝色；数量大于等于3的去B区，其他颜色去C区",
  redCount: 3,
  blueCount: 2,
  redDestination: "C",
  blueDestination: "A",
  taskObjects: [
    { category: "red", count: 3, targetRegion: "C" },
    { category: "blue", count: 2, targetRegion: "A" },
  ],
  taskRules: [
    { condition: "数量大于等于3", mapping: "满足→C，否则→A" },
  ],
  plannedGraspOrder: [
    { step: 1, category: "red", targetRegion: "C" },
    { step: 2, category: "red", targetRegion: "C" },
    { step: 3, category: "red", targetRegion: "C" },
    { step: 4, category: "blue", targetRegion: "A" },
    { step: 5, category: "blue", targetRegion: "A" },
  ],
  actualGraspOrder: [
    { step: 1, category: "red", targetRegion: "C", cubeName: "red_cube_2" },
    { step: 2, category: "red", targetRegion: "C", cubeName: "red_cube_1" },
    { step: 3, category: "red", targetRegion: "C", cubeName: "red_cube_3" },
  ],
  currentGraspStep: 4,
  currentRule: { step: 4, category: "blue", targetRegion: "A" },
  graspRuleCompliance: "IN_PROGRESS",
  selectionStrategy: "dynamic_lowest_cost",
  selectionReason: "综合成本最低：导航距离与抓取风险更低",
  selectionScore: 12.6,
  currentTaskIndex: 4,
  totalTasks: 5,
  currentAction: "前往资源点",
  robotStatus: "已释放",
  navigationStatus: "正在导航",
  armStatus: "机械臂待命",
  gripperStatus: "已释放",
  completedCount: 3,
  remainingCount: 2,
  skippedCount: 0,
  elapsedTime: 126,
  currentTargetBlock: "red_cube_3",
  currentTargetZone: "C",
  visibleTargets: ["red", "blue"],
  llmState: "LLM_OK",
  error: "",
  taskRunning: true,
  rawImageAvailable: true,
  detectionImageAvailable: true,
};

export function parseMode(value: unknown): DataMode {
  if (value === "DEMO" || value === "LIVE" || value === "AUTO") {
    return value;
  }
  return "AUTO";
}

type JsonRecord = Record<string, unknown>;

function asRecord(value: unknown): JsonRecord {
  if (value !== null && typeof value === "object" && !Array.isArray(value)) {
    return value as JsonRecord;
  }
  return {};
}

function decodeMessageRecord(value: unknown): JsonRecord {
  const envelope = asRecord(value);
  const data = envelope["data"];
  if (typeof data !== "string") {
    return envelope;
  }
  try {
    const parsed: unknown = JSON.parse(data);
    const decoded = asRecord(parsed);
    return Object.keys(decoded).length > 0 ? decoded : envelope;
  } catch {
    return envelope;
  }
}

function numberValue(value: unknown, fallback: number): number {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function stringValue(value: unknown, fallback: string): string {
  return typeof value === "string" && value.length > 0 ? value : fallback;
}

function stringArray(value: unknown, fallback: string[]): string[] {
  if (!Array.isArray(value)) {
    return fallback;
  }
  return value.filter((item): item is string => typeof item === "string");
}

function taskObjects(value: unknown, fallback: TaskObjectState[]): TaskObjectState[] {
  if (!Array.isArray(value)) {
    return fallback;
  }
  const parsed = value.flatMap((entry): TaskObjectState[] => {
    const item = asRecord(entry);
    const category = stringValue(item["category"], "");
    const count = numberValue(item["count"], 0);
    const targetRegion = stringValue(item["target_region"], "").toUpperCase();
    return category && count > 0 && ["A", "B", "C"].includes(targetRegion)
      ? [{ category, count, targetRegion }]
      : [];
  });
  return parsed.length > 0 ? parsed : fallback;
}

function taskRules(value: unknown, fallback: TaskRuleState[]): TaskRuleState[] {
  if (!Array.isArray(value)) {
    return fallback;
  }
  return value.flatMap((entry): TaskRuleState[] => {
    const item = asRecord(entry);
    const condition = stringValue(item["condition"], "");
    const mapping = stringValue(item["mapping"], "");
    return condition && mapping ? [{ condition, mapping }] : [];
  });
}

function graspOrder(value: unknown, fallback: GraspRuleStepState[]): GraspRuleStepState[] {
  if (!Array.isArray(value)) {
    return fallback;
  }
  const parsed = value.flatMap((entry): GraspRuleStepState[] => {
    const item = asRecord(entry);
    const step = numberValue(item["step"], 0);
    const category = stringValue(item["category"], "");
    const targetRegion = stringValue(item["target_region"], "").toUpperCase();
    const cubeName = stringValue(item["cube_name"], "");
    return step > 0 && category && ["A", "B", "C"].includes(targetRegion)
      ? [{ step, category, targetRegion, ...(cubeName ? { cubeName } : {}) }]
      : [];
  });
  return parsed.length > 0 ? parsed : fallback;
}

export function parseCompetitionState(message: unknown, fallback: CompetitionState): CompetitionState {
  // Live telemetry must never inherit plausible competition demo answers.
  fallback = {
    ...fallback, mode: "LIVE", question: "", redCount: 0, blueCount: 0,
    redDestination: "", blueDestination: "", taskObjects: [], taskRules: [],
    plannedGraspOrder: [], actualGraspOrder: [], currentGraspStep: 0,
    currentRule: undefined, graspRuleCompliance: "等待验证",
    currentTaskIndex: 0, totalTasks: 0, completedCount: 0, remainingCount: 0,
    skippedCount: 0, elapsedTime: 0, currentTargetBlock: "", currentTargetZone: "",
    visibleTargets: [], llmState: "waiting", error: "", taskRunning: false,
    taskId: "",
    currentAction: "等待指令", robotStatus: "等待状态", selectionReason: "",
  };
  const raw = decodeMessageRecord(message);
  const task = asRecord(raw["task"]);
  const resource = asRecord(raw["resource"] ?? raw["resources"]);
  const destination = asRecord(raw["destination"] ?? raw["target_zones"]);
  const navigation = asRecord(raw["navigation"]);
  const arm = asRecord(raw["arm"]);
  const gripper = asRecord(raw["gripper"]);

  const red = numberValue(resource["red"], fallback.redCount);
  const blue = numberValue(resource["blue"], fallback.blueCount);
  const redDestination = stringValue(destination["red"], fallback.redDestination).toUpperCase();
  const blueDestination = stringValue(destination["blue"], fallback.blueDestination).toUpperCase();
  const index = numberValue(
    raw["current_task_index"] ?? task["index"] ?? raw["current_index"],
    fallback.currentTaskIndex,
  );
  const total = numberValue(raw["total_tasks"] ?? task["total"] ?? raw["total"], fallback.totalTasks);
  const completed = numberValue(
    raw["completed_count"] ?? raw["completed"] ?? fallback.completedCount,
    fallback.completedCount,
  );
  const remaining = numberValue(raw["remaining"], Math.max(0, total - completed));
  const skipped = numberValue(raw["skipped"], fallback.skippedCount);
  const taskState = stringValue(
    raw["task_manager_state"],
    stringValue(raw["current_state"], ""),
  ).toUpperCase();
  const taskRunning = raw["task_running"] !== undefined
    ? Boolean(raw["task_running"])
    : !["", "IDLE", "COMPLETED", "FINISHED", "FAILED", "CANCELED"].includes(taskState);
  const navRaw = stringValue(navigation["state"], stringValue(raw["navigation_state"], ""));
  const armRaw = stringValue(arm["state"], stringValue(raw["arm_state"], ""));
  const gripRaw = stringValue(gripper["state"], stringValue(raw["grasp_state"], ""));
  const stateRaw = stringValue(raw["current_action"] ?? task["action"], stringValue(raw["current_state"] ?? raw["task_manager_state"], ""));
  const objects = taskObjects(raw["objects"], fallback.taskObjects);
  const rules = taskRules(raw["rules"], fallback.taskRules);
  const plannedGraspOrder = graspOrder(raw["planned_grasp_order"], fallback.plannedGraspOrder);
  const actualGraspOrder = graspOrder(raw["actual_grasp_order"], fallback.actualGraspOrder);
  const currentRule = graspOrder([raw["current_rule"]], [])[0];

  return {
    mode: fallback.mode,
    question: stringValue(raw["question"], fallback.question),
    redCount: red,
    blueCount: blue,
    redDestination: redDestination === "A" || redDestination === "B" || redDestination === "C" ? redDestination : fallback.redDestination,
    blueDestination: blueDestination === "A" || blueDestination === "B" || blueDestination === "C" ? blueDestination : fallback.blueDestination,
    taskObjects: objects,
    taskRules: rules,
    plannedGraspOrder,
    actualGraspOrder,
    currentGraspStep: numberValue(raw["current_grasp_step"], fallback.currentGraspStep),
    currentRule,
    graspRuleCompliance: stringValue(raw["grasp_rule_compliance"], fallback.graspRuleCompliance),
    selectionStrategy: stringValue(raw["selection_strategy"], fallback.selectionStrategy),
    selectionReason: stringValue(raw["selection_reason"], fallback.selectionReason),
    selectionScore: raw["selection_score"] === null || raw["selection_score"] === undefined
      ? undefined
      : numberValue(raw["selection_score"], fallback.selectionScore ?? 0),
    currentTaskIndex: index,
    totalTasks: total,
    currentAction: mapAction(stateRaw, stringValue(raw["current_target_zone"], redDestination), blueDestination, fallback.currentAction),
    robotStatus: mapRobotStatus(stateRaw, armRaw, gripRaw, navRaw, fallback.robotStatus),
    navigationStatus: mapNavigation(navRaw),
    armStatus: mapArm(armRaw),
    gripperStatus: mapGripper(gripRaw),
    completedCount: completed,
    remainingCount: remaining,
    skippedCount: skipped,
    elapsedTime: numberValue(raw["elapsed_time"] ?? raw["elapsed_sec"], fallback.elapsedTime),
    currentTargetBlock: stringValue(raw["current_target_block"], fallback.currentTargetBlock),
    currentTargetZone: stringValue(raw["current_target_zone"], fallback.currentTargetZone),
    visibleTargets: stringArray(raw["visible_targets"], fallback.visibleTargets),
    llmState: stringValue(raw["llm_state"], fallback.llmState),
    error: typeof raw["error"] === "string" ? raw["error"] : fallback.error,
    taskRunning,
    taskId: stringValue(raw["task_id"], fallback.taskId),
    rawImageAvailable: fallback.rawImageAvailable,
    detectionImageAvailable: fallback.detectionImageAvailable,
  };
}

const ACTION_LABELS: Record<string, string> = {
  idle: "等待任务",
  IDLE: "等待任务",
  PARSE_TASK: "正在解析任务",
  SELECT_TARGET: "前往资源点",
  NAVIGATE_TO_TARGET: "前往资源点",
  ALIGN_FOR_GRASP: "正在调整车体",
  GRASP: "正在抓取",
  VERIFY_GRASP: "夹取完成",
  NAVIGATE_TO_ZONE: "前往投放区",
  ALIGN_FOR_PLACE: "机械臂规划中",
  PLACE: "正在放置",
  VERIFY_PLACE: "已释放",
  NEXT_TARGET: "正在执行下一任务",
  FINISHED: "任务完成",
  RECOVERY: "正在重试",
  FAILED: "任务失败",
  navigating_startup_clearance: "启动巡检",
  navigating_pickup: "前往资源点",
  waiting_perception: "正在识别物块",
  picking: "正在抓取",
  navigating_destination: "前往投放区",
  placing: "正在放置",
  verifying: "校验结果",
  releasing: "已释放",
  completed: "任务完成",
  failed: "任务失败",
  canceled: "任务取消",
};

function mapAction(stateRaw: string, red: string, blue: string, fallback: string): string {
  if (stateRaw === "navigating_destination" || stateRaw === "NAVIGATE_TO_ZONE") {
    const zone = red || blue;
    if (zone === "A" || zone === "B" || zone === "C") {
      return `前往${zone}区`;
    }
  }
  return ACTION_LABELS[stateRaw] ?? fallback;
}

export function mapNavigation(state: string): string {
  const labels: Record<string, string> = {
    IDLE: "导航待命",
    idle: "导航待命",
    NAVIGATING: "正在导航",
    active: "正在导航",
    SUCCEEDED: "导航完成",
    succeeded: "导航完成",
    FAILED: "导航失败",
    failed: "导航失败",
    CANCELED: "导航已取消",
    canceled: "导航已取消",
  };
  return labels[state] ?? (state.length > 0 ? state : "导航待命");
}

export function mapArm(state: string): string {
  const labels: Record<string, string> = {
    IDLE: "机械臂待命",
    idle: "机械臂待命",
    PLANNING: "机械臂规划中",
    planning: "机械臂规划中",
    MOVING: "机械臂运动中",
    moving: "机械臂运动中",
    SUCCESS: "运动完成",
    success: "运动完成",
    FAILED: "机械臂规划失败",
    failed: "机械臂规划失败",
  };
  return labels[state] ?? (state.length > 0 ? state : "机械臂待命");
}

export function mapGripper(state: string): string {
  const lower = state.toLowerCase();
  if (lower.includes("attach") || lower.includes("grasp") || lower === "closed") {
    return "已夹取";
  }
  if (lower.includes("open")) {
    return "夹爪张开";
  }
  if (lower.includes("release")) {
    return "已释放";
  }
  return state.length > 0 ? state : "已释放";
}

export function mapRobotStatus(
  stateRaw: string,
  armRaw: string,
  gripRaw: string,
  navRaw: string,
  fallback: string,
): string {
  const upper = stateRaw.toUpperCase();
  if (upper.includes("FAILED") || upper.includes("TIMEOUT") || upper.includes("RECOVERY")) {
    return "机器人异常";
  }
  if (upper.includes("FINISHED") || upper.includes("COMPLETED")) {
    return "任务完成";
  }
  if (upper.includes("GRASP") || upper.includes("PICK")) {
    return "正在抓取";
  }
  if (upper.includes("PLACE")) {
    return "正在放置";
  }
  if (upper.includes("NAVIGATE")) {
    return "正在导航";
  }
  const grip = mapGripper(gripRaw);
  const arm = mapArm(armRaw);
  if (grip === "已夹取") {
    return "正在运输";
  }
  if (arm === "机械臂运动中" || arm === "机械臂规划中") {
    return arm;
  }
  const nav = mapNavigation(navRaw);
  if (nav === "正在导航") {
    return nav;
  }
  return fallback;
}
