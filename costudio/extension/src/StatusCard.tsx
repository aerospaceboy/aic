import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  RenderState,
} from "@coscene/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import { demoEngine } from "./demo";
import {
  readRoundClock,
  observeRoundStatus,
  subscribeRoundClock,
  RoundClockRecord,
} from "./roundClock";
import {
  CompetitionState,
  DEFAULT_DEMO_STATE,
  DataMode,
  MODE_LABELS,
  parseCompetitionState,
  parseMode,
} from "./state";

const FONT_FAMILY =
  "'Microsoft YaHei', 'Noto Sans CJK SC', 'PingFang SC', Arial, sans-serif";

type StatusKind =
  | "red-count"
  | "blue-count"
  | "blue-task"
  | "red-task"
  | "progress"
  | "current-task"
  | "manipulation"
  | "completion"
  | "status"
  | "timer";

type StatusCardConfig = {
  title: string;
  kind: StatusKind;
  mode: DataMode;
  topic: string;
  statusTopic?: string;
  fallback: CompetitionState;
  background: string;
  color: string;
  fontSize: number;
  fontWeight: number;
  /** 0 disables the remaining-time indicator. */
  countdownLimit: number;
};

function asConfig(value: unknown): StatusCardConfig {
  const raw = (value ?? {}) as Record<string, unknown>;
  const kind = String(raw["kind"] ?? "status") as StatusKind;
  return {
    title: String(raw["title"] ?? "状态"),
    kind,
    mode: parseMode(raw["mode"]),
    topic: String(raw["topic"] ?? "/task_manager/state"),
    statusTopic:
      typeof raw["statusTopic"] === "string" ? raw["statusTopic"] : undefined,
    fallback: { ...DEFAULT_DEMO_STATE, ...((raw["fallback"] ?? {}) as Partial<CompetitionState>) },
    background: String(raw["background"] ?? "#C1ECCA"),
    color: String(raw["color"] ?? "#191F1A"),
    fontSize: Number(raw["fontSize"] ?? 30),
    fontWeight: Number(raw["fontWeight"] ?? 700),
    countdownLimit: Number(raw["countdownLimit"] ?? 300),
  };
}

function kindText(kind: StatusKind, state: CompetitionState): string {
  switch (kind) {
    case "red-count":
      return String(state.redCount);
    case "blue-count":
      return String(state.blueCount);
    case "blue-task":
      return state.blueDestination ? `蓝色到${state.blueDestination}区` : "等待蓝色目标区";
    case "red-task":
      return state.redDestination ? `红色到${state.redDestination}区` : "等待红色目标区";
    case "progress":
      return state.currentTaskIndex > state.totalTasks
        ? "全部完成"
        : `第${state.currentTaskIndex}个`;
    case "current-task":
      return state.currentAction;
    case "manipulation":
      return state.gripperStatus;
    case "completion":
      return `已完成 ${state.completedCount}/${state.totalTasks}`;
    case "status":
      return state.robotStatus;
    case "timer":
      return "";
  }
}

/** mm:ss.d — the competition clock resolution is a tenth of a second. */
function formatClock(seconds: number): string {
  const total = Math.max(0, seconds);
  const minutes = Math.floor(total / 60);
  const whole = Math.floor(total % 60);
  const tenths = Math.floor((total - Math.floor(total)) * 10);
  return `${String(minutes).padStart(2, "0")}:${String(whole).padStart(2, "0")}.${tenths}`;
}

/** Pick a readable note colour for the configured card background. */
function noteColor(background: string): string {
  const match = /^#?([0-9a-fA-F]{6})$/.exec(background.trim());
  if (!match) {
    return "#666666";
  }
  const value = parseInt(match[1] ?? "000000", 16);
  const r = (value >> 16) & 0xff;
  const g = (value >> 8) & 0xff;
  const b = value & 0xff;
  const luminance = (0.299 * r + 0.587 * g + 0.114 * b) / 255;
  return luminance < 0.55 ? "#D7DEE8" : "#5A6472";
}

/** mm:ss without tenths, for the remaining-time line. */
function formatShort(seconds: number): string {
  const total = Math.max(0, Math.ceil(seconds));
  return `${String(Math.floor(total / 60)).padStart(2, "0")}:${String(total % 60).padStart(2, "0")}`;
}

/** Read the durable click session; ROS/simulation time never starts the clock. */
function useRoundTimer(active: boolean): {
  elapsed: number; phase: RoundClockRecord["phase"] | "idle";
} {
  const [record, setRecord] = useState(() => readRoundClock());
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    if (!active) { return undefined; }
    const update = (): void => { setRecord(readRoundClock()); setNow(Date.now()); };
    const unsubscribe = subscribeRoundClock(update);
    update();
    const id = window.setInterval(update, 100);
    return () => { window.clearInterval(id); unsubscribe(); };
  }, [active]);
  return {
    elapsed: record ? Math.max(0, (record.stoppedAt ?? now) - record.startedAt) / 1000 : 0,
    phase: record?.phase ?? "idle",
  };
}
function statusBackground(state: CompetitionState, fallback: string): string {
  const status = state.robotStatus;
  if (/失败|异常|超时|碰撞/.test(status)) {
    return "#F6CDCD";
  }
  if (/运行|导航|抓取|放置|运输|识别|规划|避障/.test(status)) {
    return "#CFE4F6";
  }
  if (/等待/.test(status)) {
    return "#E9E9E9";
  }
  return fallback;
}

function StatusCard({
  context,
}: {
  context: PanelExtensionContext;
}): ReactElement {
  const [config, setConfig] = useState<StatusCardConfig>(() =>
    asConfig(context.initialState),
  );
  const [liveMessage, setLiveMessage] = useState<unknown>();
  const [lastLiveAt, setLastLiveAt] = useState<number>(0);
  const [demoState, setDemoState] = useState<CompetitionState>({
    ...DEFAULT_DEMO_STATE,
  });
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [, setTick] = useState(0);
  const cardRef = useRef<HTMLDivElement>(null);
  const [cardHeight, setCardHeight] = useState(60);

  useLayoutEffect(() => {
    const element = cardRef.current;
    if (!element) { return undefined; }
    const observer = new ResizeObserver(([entry]) => {
      if (entry) { setCardHeight(entry.contentRect.height); }
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  useLayoutEffect(() => {
    context.watch("currentFrame");
    context.watch("topics");
    context.onRender = (renderState: Immutable<RenderState>, done: () => void) => {
      setRenderDone(() => done);
      const frame = renderState.currentFrame ?? [];
      let latest: MessageEvent | undefined;
      for (const event of frame) {
        if (event.topic === config.topic || event.topic === config.statusTopic) {
          latest = event;
        }
      }
      if (latest !== undefined) {
        if (config.kind === "timer" && config.mode !== "DEMO") {
          observeRoundStatus(latest.message);
        }
        setLiveMessage(latest.message);
        setLastLiveAt(Date.now());
      }
    };
    const subscriptions = [
      { topic: config.topic },
      ...(config.statusTopic ? [{ topic: config.statusTopic }] : []),
    ];
    context.subscribe(subscriptions);
    context.setDefaultPanelTitle(config.title);
  }, [config, context]);

  useEffect(() => {
    return demoEngine.subscribe(setDemoState);
  }, []);

  useEffect(() => {
    const timer = setInterval(() => {
      setTick((value) => value + 1);
    }, 1000);
    return () => {
      clearInterval(timer);
    };
  }, []);

  useEffect(() => {
    renderDone?.();
  }, [renderDone, liveMessage, demoState]);

  useEffect(() => {
    context.saveState(config);
    context.updatePanelSettingsEditor({
      actionHandler: (action) => {
        if (action.action === "update" && action.payload.path.length === 2) {
          const key = action.payload.path[1];
          const value = typeof action.payload.value === "string" ? action.payload.value : undefined;
          if (key === "mode" && value !== undefined) {
            setConfig((old) => ({ ...old, mode: parseMode(value) }));
          } else if (key === "topic" && value !== undefined) {
            setConfig((old) => ({ ...old, topic: value }));
          }
        }
      },
      nodes: {
        general: {
          label: "面板设置",
          fields: {
            mode: {
              label: "数据模式",
              input: "select",
              value: config.mode,
              options: [
                { label: MODE_LABELS.AUTO, value: "AUTO" },
                { label: MODE_LABELS.DEMO, value: "DEMO" },
                { label: MODE_LABELS.LIVE, value: "LIVE" },
              ],
            },
            topic: {
              label: "任务状态话题",
              input: "string",
              value: config.topic,
            },
          },
        },
      },
    });
  }, [config, context]);

  const hasRecentLive = Date.now() - lastLiveAt < 2500;
  const effectiveMode: DataMode =
    config.mode === "AUTO" ? (hasRecentLive ? "LIVE" : "DEMO") : config.mode;
  const state = useMemo<CompetitionState | undefined>(() => {
    if (effectiveMode === "DEMO") {
      return demoState;
    }
    if (liveMessage === undefined) {
      return undefined;
    }
    return parseCompetitionState(liveMessage, demoState);
  }, [demoState, effectiveMode, liveMessage]);

  const timer = useRoundTimer(config.kind === "timer");
  const overLimit =
    config.countdownLimit > 0 && timer.elapsed > config.countdownLimit;
  const timerNote =
    timer.phase === "idle"
      ? "等待点击“确定并执行”"
      : timer.phase === "completed"
        ? "任务完成 · 已冻结"
        : timer.phase === "failed" || timer.phase === "canceled"
          ? "任务失败/取消 · 已冻结"
        : timer.phase === "pending"
          ? "计时中 · 等待解析/执行"
        : overLimit
          ? `已超时 ${formatShort(timer.elapsed - config.countdownLimit)}`
          : config.countdownLimit > 0
            ? `剩余 ${formatShort(config.countdownLimit - timer.elapsed)}`
            : "计时中";

  const text =
    config.kind === "timer"
      ? formatClock(timer.elapsed)
      : state === undefined
        ? config.mode === "LIVE"
          ? "等待 ROS2 数据..."
          : kindText(config.kind, demoState)
        : kindText(config.kind, state);
  const background =
    config.kind === "timer" && overLimit
      ? "#F6CDCD"
      : config.kind === "status" && state !== undefined
        ? statusBackground(state, config.background)
        : config.background;
  // Small desktop windows must not clip either the stopwatch or its freeze
  // label. Secondary status notes are omitted only when their card is tiny.
  const showNote = config.kind === "timer" || cardHeight >= 40;
  const hasNote = showNote && ["timer", "progress", "manipulation", "completion"].includes(config.kind);
  const fittedFontSize = Math.min(config.fontSize,
    Math.max(10, Math.floor((cardHeight - (hasNote ? 16 : 0)) / 1.1)));

  return (
    <div
      ref={cardRef}
      style={{
        alignItems: "center",
        background,
        boxSizing: "border-box",
        color: config.kind === "timer" && overLimit ? "#8A1C1C" : config.color,
        display: "flex",
        flexDirection: "column",
        fontFamily: FONT_FAMILY,
        fontWeight: config.fontWeight,
        height: "100%",
        justifyContent: "center",
        margin: 0,
        overflow: "hidden",
        padding: 0,
        textAlign: "center",
        width: "100%",
      }}
    >
      <div
        style={{
          fontSize: fittedFontSize,
          lineHeight: 1.1,
          whiteSpace: "nowrap",
        }}
      >
        {text}
      </div>
      {showNote && config.kind === "progress" && state !== undefined ? (
        <div style={{ color: "#666666", fontSize: 12, fontWeight: 400, marginTop: 2 }}>
          当前 {state.currentTaskIndex} / {state.totalTasks}
        </div>
      ) : undefined}
      {showNote && config.kind === "manipulation" && state !== undefined ? (
        <div style={{ color: "#666666", fontSize: 12, fontWeight: 400, marginTop: 2 }}>
          {state.armStatus}
        </div>
      ) : undefined}
      {showNote && config.kind === "completion" && state !== undefined ? (
        <div style={{ color: "#666666", fontSize: 12, fontWeight: 400, marginTop: 2 }}>
          剩余 {state.remainingCount} · 跳过 {state.skippedCount}
        </div>
      ) : undefined}
      {config.kind === "timer" ? (
        <div
          style={{
            color: noteColor(background),
            fontSize: 12,
            fontWeight: 400,
            marginTop: 2,
          }}
        >
          {timerNote}
        </div>
      ) : undefined}
    </div>
  );
}

export function initStatusCard(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<StatusCard context={context} />);
  return () => {
    root.unmount();
  };
}
