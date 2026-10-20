import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  RenderState,
} from "@coscene/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import { demoEngine } from "./demo";
import { startButtonView } from "./startButton";
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

type StartButtonConfig = {
  title: string;
  mode: DataMode;
  topic: string;
  statusTopic: string;
};

function asConfig(value: unknown): StartButtonConfig {
  const raw = (value ?? {}) as Record<string, unknown>;
  return {
    title: String(raw["title"] ?? "开始任务"),
    mode: parseMode(raw["mode"]),
    topic: String(raw["topic"] ?? "/competition/mission/start"),
    statusTopic: String(raw["statusTopic"] ?? "/task_manager/state"),
  };
}

function StartButton({
  context,
}: {
  context: PanelExtensionContext;
}): ReactElement {
  const [config, setConfig] = useState<StartButtonConfig>(() =>
    asConfig(context.initialState),
  );
  const [canPublish] = useState<boolean>(
    () =>
      typeof context.advertise === "function" &&
      typeof context.publish === "function",
  );
  const [liveMessage, setLiveMessage] = useState<unknown>();
  const [lastLiveAt, setLastLiveAt] = useState<number>(0);
  const [demoState, setDemoState] = useState<CompetitionState>({
    ...DEFAULT_DEMO_STATE,
  });
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [, setTick] = useState(0);

  useLayoutEffect(() => {
    context.watch("currentFrame");
    context.watch("topics");
    context.onRender = (renderState: Immutable<RenderState>, done: () => void) => {
      setRenderDone(() => done);
      const frame = renderState.currentFrame ?? [];
      let latest: MessageEvent | undefined;
      for (const event of frame) {
        if (event.topic === config.statusTopic) {
          latest = event;
        }
      }
      if (latest !== undefined) {
        setLiveMessage(latest.message);
        setLastLiveAt(Date.now());
      }
    };
    context.subscribe([{ topic: config.statusTopic }]);
    context.setDefaultPanelTitle(config.title);
    return () => {
      if (canPublish) {
        context.unadvertise?.(config.topic);
      }
    };
  }, [canPublish, config, context]);

  useEffect(() => {
    if (canPublish) {
      context.advertise?.(config.topic, "std_msgs/Bool");
    }
  }, [canPublish, config, context]);

  useEffect(() => {
    return demoEngine.subscribe(setDemoState);
  }, []);

  useEffect(() => {
    const timer = setInterval(() => setTick((value) => value + 1), 1000);
    return () => clearInterval(timer);
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
          if (key === "mode" && typeof action.payload.value === "string") {
            setConfig((old) => ({ ...old, mode: parseMode(action.payload.value) }));
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
              label: "任务启动话题",
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
  const state: CompetitionState | undefined =
    effectiveMode === "DEMO"
      ? demoState
      : liveMessage === undefined
        ? undefined
        : parseCompetitionState(liveMessage, demoState);
  const view = startButtonView(
    effectiveMode,
    state,
    demoEngine.isRunning(),
    canPublish,
  );

  const clickStart = (): void => {
    if (effectiveMode === "DEMO") {
      if (demoEngine.isRunning()) {
        demoEngine.stop();
      } else {
        demoEngine.start();
      }
      return;
    }
    if (view.disabled) {
      return;
    }
    context.publish?.(config.topic, { data: true });
  };

  const hint = !canPublish
    ? "当前数据源不支持任务启动"
    : effectiveMode === "DEMO"
      ? view.running
        ? "演示任务运行中，点击可停止演示"
        : "演示模式：点击后运行演示任务，不会控制真实机器人"
      : state === undefined
        ? "尚未连接 ROS2 数据源"
        : view.running
          ? "任务运行中"
          : view.finished
            ? "任务已完成"
            : "任务系统已就绪";

  return (
    <div
      style={{
        alignItems: "center",
        background: "#F4F4F5",
        boxSizing: "border-box",
        display: "flex",
        flexDirection: "column",
        fontFamily: FONT_FAMILY,
        height: "100%",
        justifyContent: "center",
        margin: 0,
        padding: "6px 8px",
        width: "100%",
      }}
    >
      <button
        disabled={view.disabled}
        onClick={clickStart}
        style={{
          background: view.disabled && !view.running ? "#E5E5E5" : view.running ? "#C1ECCA" : "#FFFFFF",
          border: "1px solid #D3D3D3",
          borderRadius: 3,
          color: view.disabled && !view.running ? "#999999" : "#191F1A",
          cursor: view.running ? "not-allowed" : view.disabled ? "not-allowed" : "pointer",
          fontFamily: FONT_FAMILY,
          fontSize: 13,
          height: 30,
          margin: 0,
          padding: 0,
          width: 78,
        }}
        type="button"
      >
        {view.label}
      </button>
      <div
        style={{
          color: "#888888",
          fontSize: 11,
          marginTop: 6,
          maxWidth: 220,
          textAlign: "center",
        }}
      >
        {hint}
      </div>
    </div>
  );
}

export function initStartButton(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<StartButton context={context} />);
  return () => {
    root.unmount();
  };
}
