import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  RenderState,
} from "@coscene/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import { demoEngine } from "./demo";
import { splitProblemChunks } from "./chunk";
import { finishRoundSubmission, markRoundStart, observeRoundStatus } from "./roundClock";
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
const PROBLEM_CHUNK_SUFFIX = "_chunk";
const PROBLEM_CHUNK_INTERVAL_MS = 80;

type TopicAreaConfig = {
  title: string;
  mode: DataMode;
  topic: string;
  statusTopic?: string;
  problemTopic: string;
};

function asConfig(value: unknown): TopicAreaConfig {
  const raw = (value ?? {}) as Record<string, unknown>;
  return {
    title: String(raw["title"] ?? "任务信息"),
    mode: parseMode(raw["mode"]),
    topic: String(raw["topic"] ?? "/task_manager/state"),
    statusTopic:
      typeof raw["statusTopic"] === "string" ? raw["statusTopic"] : undefined,
    problemTopic: String(raw["problemTopic"] ?? "/competition/task/problem"),
  };
}

function colorLabel(value: string): string {
  const lower = value.toLowerCase();
  if (lower === "red") {
    return "红";
  }
  if (lower === "blue") {
    return "蓝";
  }
  return value;
}

function formatObjects(state: CompetitionState): string {
  return state.taskObjects.length > 0
    ? state.taskObjects
        .map((item) => `${colorLabel(item.category)} ${item.count}个→${item.targetRegion}区`)
        .join(" · ")
    : "等待大模型解析";
}

function formatRules(state: CompetitionState): string {
  return state.taskRules.length > 0
    ? state.taskRules
        .map((rule) => `${rule.condition}（${rule.mapping}）`)
        .join("；")
    : "无附加规则";
}

function formatGraspOrder(state: CompetitionState): string {
  return state.plannedGraspOrder.length > 0
    ? state.plannedGraspOrder
        .map((step) => `${step.step}.${colorLabel(step.category)}→${step.targetRegion}`)
        .join(" · ")
    : "等待大模型抓取规则";
}

function formatActualOrder(state: CompetitionState): string {
  return state.actualGraspOrder.length > 0
    ? state.actualGraspOrder
        .map((step) => `${step.step}.${step.cubeName ?? colorLabel(step.category)}→${step.targetRegion}✓`)
        .join(" · ")
    : "尚无已完成抓取";
}

function formatElapsed(seconds: number): string {
  const safe = Math.max(0, Math.floor(seconds));
  return `${String(Math.floor(safe / 60)).padStart(2, "0")}:${String(safe % 60).padStart(2, "0")}`;
}

async function waitFor(milliseconds: number): Promise<void> {
  await new Promise<void>((resolve) => setTimeout(resolve, milliseconds));
}

function TopicArea({
  context,
}: {
  context: PanelExtensionContext;
}): ReactElement {
  const [config, setConfig] = useState<TopicAreaConfig>(() =>
    asConfig(context.initialState),
  );
  const [canPublish] = useState<boolean>(
    () =>
      typeof context.advertise === "function" &&
      typeof context.publish === "function",
  );
  const [questionInput, setQuestionInput] = useState("");
  const [mappingInput, setMappingInput] = useState("");
  const [submissionLocked, setSubmissionLocked] = useState(false);
  const [submissionMessage, setSubmissionMessage] = useState("");
  const [submittedQuestion, setSubmittedQuestion] = useState("");
  const [submittedId, setSubmittedId] = useState("");
  // Only the newest click may keep publishing chunks; editing the form cancels
  // an in-flight submission instead of letting two rounds interleave on the wire.
  const activeSubmission = useRef("");
  const [liveMessage, setLiveMessage] = useState<unknown>();
  const [lastLiveAt, setLastLiveAt] = useState<number>(0);
  const [demoState, setDemoState] = useState<CompetitionState>({
    ...DEFAULT_DEMO_STATE,
  });
  const [demoRunning, setDemoRunning] = useState<boolean>(false);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [, setTick] = useState(0);
  const problemChunkTopic = `${config.problemTopic}${PROBLEM_CHUNK_SUFFIX}`;

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
        if (config.mode !== "DEMO") { observeRoundStatus(latest.message); }
        setLiveMessage(latest.message);
        setLastLiveAt(Date.now());
      }
    };
    context.subscribe([
      { topic: config.topic },
      ...(config.statusTopic ? [{ topic: config.statusTopic }] : []),
    ]);
    context.setDefaultPanelTitle(config.title);
  }, [config, context]);

  useEffect(() => {
    return demoEngine.subscribe(setDemoState);
  }, []);

  useEffect(() => {
    if (!canPublish) {
      return;
    }
    context.advertise?.(problemChunkTopic, "std_msgs/String");
    return () => {
      context.unadvertise?.(problemChunkTopic);
    };
  }, [canPublish, context, problemChunkTopic]);

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
          const value = typeof action.payload.value === "string" ? action.payload.value : undefined;
          if (key === "mode" && value !== undefined) {
            setConfig((old) => ({ ...old, mode: parseMode(value) }));
          } else if (key === "topic" && value !== undefined) {
            setConfig((old) => ({ ...old, topic: value }));
          } else if (key === "statusTopic" && value !== undefined) {
            setConfig((old) => ({ ...old, statusTopic: value }));
          } else if (key === "problemTopic" && value !== undefined) {
            setConfig((old) => ({ ...old, problemTopic: value }));
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
            statusTopic: {
              label: "备用状态话题",
              input: "string",
              value: config.statusTopic ?? "",
            },
            problemTopic: {
              label: "题目提交话题",
              input: "string",
              value: config.problemTopic,
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

  let evidence: Record<string, unknown> = {};
  try {
    const envelope = liveMessage as { data?: string } | undefined;
    const status = JSON.parse(envelope?.data ?? "{}");
    evidence = status.parse_result ?? {};
  } catch { /* Invalid telemetry remains visibly unverified. */ }
  const currentEvidence = hasRecentLive && effectiveMode === "LIVE" &&
    (!submittedId || evidence["request_id"] === submittedId);
  const parseState = currentEvidence ? String(evidence["state"] ?? "waiting") : "waiting";
  const published = parseState === "published" && evidence["published"] === true;
  const modelVerified = published && evidence["source"] === "llm";
  const parseLabel = !hasRecentLive ? "连接中断 / 等待实时数据" :
    modelVerified ? "大模型解析成功 · 指令已发布" :
    parseState === "failed" ? "解析失败 · 未发布本题指令" :
    parseState === "parsing" ? "已接收题目 · 大模型解析中" :
    published ? "规则解析结果 · 非大模型结果" : "等待本题解析结果";

  useEffect(() => {
    if (!submissionLocked || !submittedQuestion) {
      return;
    }
    if (currentEvidence && parseState === "failed") {
      setSubmissionLocked(false);
      setSubmissionMessage(`解析失败：${String(evidence["error"] ?? "请检查题目及映射规则")}`);
      return;
    }
    if (currentEvidence && published) {
      setSubmissionLocked(false);
      setSubmissionMessage(parseLabel);
      return;
    }
    if (currentEvidence && parseState === "parsing") {
      setSubmissionMessage("后端已接收，正在进行大模型解析...");
      return;
    }
    const timeout = setTimeout(() => {
      setSubmissionLocked(false);
      setSubmissionMessage("后端未收到题目，请检查连接后重试");
    }, 8000);
    return () => clearTimeout(timeout);
  }, [parseState, currentEvidence, published, parseLabel, submissionLocked, submittedQuestion]);

  const rosConnected = effectiveMode === "LIVE" && hasRecentLive;
  const taskRunning = state?.taskRunning === true;
  const inputsReady =
    questionInput.trim().length > 0 && mappingInput.trim().length > 0;
  const canSubmit =
    canPublish &&
    rosConnected &&
    inputsReady &&
    !taskRunning &&
    !submissionLocked;

  const editQuestion = (value: string): void => {
    activeSubmission.current = "";
    setQuestionInput(value);
    setSubmissionLocked(false);
    setSubmissionMessage("");
    setSubmittedQuestion("");
    setSubmittedId("");
  };

  const editMapping = (value: string): void => {
    activeSubmission.current = "";
    setMappingInput(value);
    setSubmissionLocked(false);
    setSubmissionMessage("");
    setSubmittedQuestion("");
    setSubmittedId("");
  };

  const submitProblem = async (): Promise<void> => {
    const clickedAt = Date.now();
    const question = questionInput.trim();
    const mappingRules = mappingInput.trim();
    if (!question || !mappingRules) {
      setSubmissionMessage("请完整填写题目和区域映射规则");
      return;
    }
    if (!canSubmit) {
      return;
    }
    const submissionId = `${clickedAt.toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
    markRoundStart(submissionId, clickedAt);
    try {
      const chunks = splitProblemChunks(
        submissionId,
        JSON.stringify({
          task_id: submissionId,
          question,
          mapping_rules: mappingRules,
        }),
      );
      activeSubmission.current = submissionId;
      setSubmissionLocked(true);
      setSubmittedQuestion(question);
      setSubmittedId(submissionId);
      setSubmissionMessage(`正在发送题目分片 1/${chunks.length}...`);
      // The competition clock starts at submission, not when the base moves:
      // this timestamp therefore also covers the model parsing latency.
      for (const chunk of chunks) {
        if (activeSubmission.current !== submissionId) {
          // The operator edited the form while chunks were in flight. Abandon
          // this submission instead of mixing two rounds' chunks.
          finishRoundSubmission(submissionId);
          return;
        }
        context.publish?.(problemChunkTopic, { data: JSON.stringify(chunk) });
        if (chunk.i + 1 < chunks.length) {
          await waitFor(PROBLEM_CHUNK_INTERVAL_MS);
        }
      }
      if (activeSubmission.current === submissionId) {
        setSubmissionMessage("已发送，正在等待后端接收...");
      }
    } catch {
      finishRoundSubmission(submissionId);
      setSubmissionLocked(false);
      setSubmittedQuestion("");
      setSubmissionMessage("提交失败，请检查 ROS2 连接后重试");
    }
  };

  const toggleDemo = (): void => {
    if (demoEngine.isRunning()) {
      demoEngine.stop();
      setDemoRunning(false);
    } else {
      demoEngine.start();
      setDemoRunning(true);
    }
  };

  return (
    <div
      style={{
        background: "#F4F4F5",
        boxSizing: "border-box",
        display: "flex",
        flexDirection: "column",
        fontFamily: FONT_FAMILY,
        height: "100%",
        margin: 0,
        padding: 0,
        width: "100%",
      }}
    >
      <div
        style={{
          alignItems: "center",
          background: "#FFFFFF",
          borderBottom: "1px solid #D3D3D3",
          boxSizing: "border-box",
          color: "#333333",
          display: "flex",
          flex: "0 0 27px",
          fontSize: 12,
          gap: 6,
          margin: 0,
          padding: "0 8px",
          width: "100%",
        }}
      >
        <svg height={14} viewBox="0 0 14 14" width={14}>
          <path d="M2 1.5h6l4 4v7a0.5 0.5 0 0 1-0.5 0.5h-9A0.5 0.5 0 0 1 2 12.5z" fill="none" stroke="#555555" strokeWidth={1} />
          <path d="M8 1.5v4h4" fill="none" stroke="#555555" strokeWidth={1} />
        </svg>
        <div style={{ flex: 1, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
          {config.topic}
        </div>
        <span style={{ color: "#666666", fontSize: 13 }}>⚙</span>
        <span style={{ color: "#666666", fontSize: 14 }}>⋮</span>
      </div>
      <div
        style={{
          alignItems: "flex-start",
          background: "#F4F4F5",
          color: "#191F1A",
          display: "flex",
          flex: 1,
          fontSize: 12,
          justifyContent: "center",
          margin: 0,
          overflow: "auto",
          padding: "6px 10px",
          width: "100%",
        }}
      >
        <div
          style={{
            display: "flex",
            flexDirection: "column",
            gap: 4,
            width: "100%",
          }}
        >
          <div
            style={{
              alignItems: "center",
              display: "grid",
              gap: "5px 8px",
              gridTemplateColumns: "72px minmax(0, 1fr) 104px",
            }}
          >
            <label htmlFor="competition-question" style={{ fontWeight: 700 }}>
              输入题目
            </label>
            <textarea
              id="competition-question"
              onChange={(event) => editQuestion(event.target.value)}
              placeholder="请输入完整比赛题目"
              style={{
                background: "#FFFFFF",
                border: "1px solid #B8B8B8",
                borderRadius: 3,
                boxSizing: "border-box",
                fontFamily: FONT_FAMILY,
                fontSize: 12,
                gridColumn: "2 / 4",
                height: 42,
                lineHeight: 1.35,
                padding: "4px 6px",
                resize: "none",
                width: "100%",
              }}
              value={questionInput}
            />
            <label htmlFor="competition-mapping" style={{ fontWeight: 700 }}>
              区域映射
            </label>
            <input
              id="competition-mapping"
              onChange={(event) => editMapping(event.target.value)}
              placeholder="例如：x代表红色，y代表蓝色，数量≥3去C区，其他去A区"
              style={{
                background: "#FFFFFF",
                border: "1px solid #B8B8B8",
                borderRadius: 3,
                boxSizing: "border-box",
                fontFamily: FONT_FAMILY,
                fontSize: 12,
                height: 28,
                padding: "0 6px",
                width: "100%",
              }}
              value={mappingInput}
            />
            <button
              disabled={!canSubmit}
              onClick={() => {
                void submitProblem();
              }}
              style={{
                background: canSubmit ? "#2F6FED" : "#D7D7D7",
                border: "none",
                borderRadius: 3,
                color: canSubmit ? "#FFFFFF" : "#777777",
                cursor: canSubmit ? "pointer" : "not-allowed",
                fontFamily: FONT_FAMILY,
                fontSize: 12,
                fontWeight: 700,
                height: 28,
              }}
              type="button"
            >
              {submissionLocked ? "已提交" : "确定并执行"}
            </button>
          </div>
          <div
            style={{
              color: submissionMessage.includes("失败") ||
                submissionMessage.includes("完整") ||
                submissionMessage.includes("未收到")
                ? "#B42318"
                : "#176B3A",
              fontSize: 11,
              minHeight: 14,
              paddingLeft: 80,
            }}
          >
            {submissionMessage ||
              (!canPublish
                ? "当前数据源不支持提交题目"
                : !rosConnected
                  ? "等待 ROS2 连接后即可提交"
                  : taskRunning
                    ? "当前任务运行中，完成后可提交新题目"
                    : "填写两项内容后点击“确定并执行”")}
          </div>
          <div
            style={{
              alignItems: "center",
              display: "flex",
              justifyContent: "space-between",
            }}
          >
            <span style={{ color: "#666666" }}>
              {rosConnected ? "ROS2：● 已连接" : "ROS2：○ 未连接"}
            </span>
            <span style={{ color: "#666666" }}>
              {effectiveMode === "DEMO"
                ? "演示模式"
                : effectiveMode === "LIVE"
                  ? "实时模式"
                  : "自动模式"}
            </span>
          </div>
          <div
            style={{
              fontSize: 13,
              fontWeight: 700,
              overflow: "hidden",
              textOverflow: "ellipsis",
              whiteSpace: "nowrap",
            }}
            title={state?.question}
          >
            {state === undefined
              ? "等待 ROS2 数据..."
              : `比赛题目：${state.question || "等待大模型解析"}`}
          </div>
          <div style={{ background: modelVerified ? "#DCFCE7" : "#FFF1E6", padding: 8, borderRadius: 4, color: modelVerified ? "#166534" : "#9A3412", fontWeight: 700 }}>
            {parseLabel}
            <div style={{ fontSize: 11, fontWeight: 400 }}>
              {currentEvidence ? `模型：${String(evidence["model"] ?? "—")} · 任务：${String(evidence["task_id"] || evidence["request_id"] || "—")}` : "等待关联的任务编号"}
            </div>
          </div>
          <div style={{ color: "#333333" }}>
            {state === undefined
              ? "尚未收到任务状态消息"
              : `指令内容：${published ? formatObjects(state) : "尚无本题有效指令"}`}
          </div>
          <div style={{ color: "#333333" }}>
            {state === undefined
              ? "任务规则：等待大模型解析"
              : `任务规则：${formatRules(state)}`}
          </div>
          <div
            style={{ color: "#333333", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}
            title={state === undefined ? undefined : formatGraspOrder(state)}
          >
            {state === undefined
              ? "大模型抓取规则：等待解析"
              : `题目展开：${published ? formatGraspOrder(state) : "等待本题解析"}`}
          </div>
          <div style={{ color: "#333333" }}>
            {state === undefined
              ? "当前执行：等待规则"
              : `当前执行：${state.currentGraspStep}/${state.totalTasks} · ${state.currentRule ? `${colorLabel(state.currentRule.category)}→${state.currentRule.targetRegion}` : "已完成"} · 实际目标：${state.currentTargetBlock || "待选择"}`}
          </div>
          <div
            style={{ color: state?.graspRuleCompliance === "RULE_VIOLATION" ? "#B42318" : "#176B3A", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}
            title={state === undefined ? undefined : formatActualOrder(state)}
          >
            {state === undefined
              ? "规则一致性：等待验证"
              : `已执行：${formatActualOrder(state)} · 规则一致性：${state.graspRuleCompliance}`}
          </div>
          <div
            style={{
              color: "#333333",
              overflow: "hidden",
              textOverflow: "ellipsis",
              whiteSpace: "nowrap",
            }}
            title={state?.selectionReason}
          >
            {state === undefined
              ? "动态策略：等待运行时评估"
              : `动态策略：综合成本最优 · ${state.selectionReason || "每项任务前重新计算"}${state.selectionScore === undefined ? "" : ` · ${state.selectionScore.toFixed(2)}`}`}
          </div>
          <div
            style={{
              color: state?.error ? "#B42318" : "#666666",
              overflow: "hidden",
              textOverflow: "ellipsis",
              whiteSpace: "nowrap",
            }}
          >
            {state === undefined
              ? "识别结果与当前执行状态将在这里显示"
              : state.error
                ? `异常：${state.error}`
                : `当前目标：${state.currentTargetBlock || "待选择"} → ${state.currentTargetZone || "?"}区 · 识别：${state.visibleTargets.length > 0 ? state.visibleTargets.map(colorLabel).join("、") : "暂无目标"} · 运动耗时：${formatElapsed(state.elapsedTime)} · ${state.llmState}`}
          </div>
          <div style={{ display: "flex", gap: 6, justifyContent: "center" }}>
            <button
              onClick={toggleDemo}
              style={{
                background: "#FFFFFF",
                border: "1px solid #D3D3D3",
                borderRadius: 3,
                color: "#191F1A",
                fontFamily: FONT_FAMILY,
                fontSize: 11,
                height: 20,
                padding: "0 8px",
              }}
              type="button"
            >
              {demoRunning ? "停止演示" : "开始演示"}
            </button>
            <select
              onChange={(event) =>
                setConfig((old) => ({
                  ...old,
                  mode: parseMode(event.target.value),
                }))
              }
              style={{
                background: "#FFFFFF",
                border: "1px solid #D3D3D3",
                borderRadius: 3,
                color: "#191F1A",
                fontFamily: FONT_FAMILY,
                fontSize: 11,
                height: 20,
              }}
              value={config.mode}
            >
              <option value="AUTO">{MODE_LABELS.AUTO}</option>
              <option value="DEMO">{MODE_LABELS.DEMO}</option>
              <option value="LIVE">{MODE_LABELS.LIVE}</option>
            </select>
          </div>
        </div>
      </div>
    </div>
  );
}

export function initTopicArea(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<TopicArea context={context} />);
  return () => {
    root.unmount();
  };
}
