import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  RenderState,
} from "@coscene/extension";
import {
  ReactElement,
  useEffect,
  useLayoutEffect,
  useRef,
  useState,
} from "react";
import { createRoot } from "react-dom/client";

import {
  DataMode,
  MODE_LABELS,
  parseMode,
} from "./state";

const FONT_FAMILY =
  "'Microsoft YaHei', 'Noto Sans CJK SC', 'PingFang SC', Arial, sans-serif";

type VisionVariant = "raw" | "detection";

type VisionImageConfig = {
  title: string;
  variant: VisionVariant;
  mode: DataMode;
  topic: string;
  background: string;
  placeholder: string;
};

function asConfig(value: unknown): VisionImageConfig {
  const raw = (value ?? {}) as Record<string, unknown>;
  const variant = raw["variant"] === "detection" ? "detection" : "raw";
  return {
    title: String(raw["title"] ?? "相机画面"),
    variant,
    mode: parseMode(raw["mode"]),
    topic: String(raw["topic"] ?? "/camera/color/image_raw"),
    background: String(raw["background"] ?? "#F4F4F5"),
    placeholder: String(raw["placeholder"] ?? "等待相机图像..."),
  };
}

type RosImage = {
  width?: number;
  height?: number;
  encoding?: string;
  data?: unknown;
};

function bytesOf(message: unknown): Uint8Array | undefined {
  const data = (message as RosImage).data;
  if (data instanceof Uint8Array) {
    return data;
  }
  if (Array.isArray(data)) {
    return new Uint8Array(data);
  }
  if (data instanceof ArrayBuffer) {
    return new Uint8Array(data);
  }
  return undefined;
}

function imageDataOf(message: unknown): ImageData | undefined {
  const image = (message ?? {}) as RosImage;
  const width = Number(image.width ?? 0);
  const height = Number(image.height ?? 0);
  const encoding = String(image.encoding ?? "");
  const bytes = bytesOf(message);
  if (width <= 0 || height <= 0 || bytes === undefined) {
    return undefined;
  }
  const size = width * height;
  if (encoding !== "mono8" && bytes.length < size * 3) {
    return undefined;
  }
  const rgba = new Uint8ClampedArray(size * 4);
  if (encoding === "rgb8" || encoding === "8UC3") {
    for (let i = 0; i < size; i += 1) {
      const s = i * 3;
      const t = i * 4;
      rgba[t] = bytes[s] ?? 0;
      rgba[t + 1] = bytes[s + 1] ?? 0;
      rgba[t + 2] = bytes[s + 2] ?? 0;
      rgba[t + 3] = 255;
    }
  } else if (encoding === "bgr8") {
    for (let i = 0; i < size; i += 1) {
      const s = i * 3;
      const t = i * 4;
      rgba[t] = bytes[s + 2] ?? 0;
      rgba[t + 1] = bytes[s + 1] ?? 0;
      rgba[t + 2] = bytes[s] ?? 0;
      rgba[t + 3] = 255;
    }
  } else if (encoding === "rgba8" || encoding === "8UC4") {
    for (let i = 0; i < size; i += 1) {
      const s = i * 4;
      rgba[s] = bytes[s] ?? 0;
      rgba[s + 1] = bytes[s + 1] ?? 0;
      rgba[s + 2] = bytes[s + 2] ?? 0;
      rgba[s + 3] = bytes[s + 3] ?? 255;
    }
  } else if (encoding === "bgra8") {
    for (let i = 0; i < size; i += 1) {
      const s = i * 4;
      rgba[s] = bytes[s + 2] ?? 0;
      rgba[s + 1] = bytes[s + 1] ?? 0;
      rgba[s + 2] = bytes[s] ?? 0;
      rgba[s + 3] = bytes[s + 3] ?? 255;
    }
  } else if (encoding === "mono8" || encoding === "8UC1") {
    for (let i = 0; i < size; i += 1) {
      const g = bytes[i] ?? 0;
      const t = i * 4;
      rgba[t] = g;
      rgba[t + 1] = g;
      rgba[t + 2] = g;
      rgba[t + 3] = 255;
    }
  } else {
    return undefined;
  }
  return new ImageData(rgba, width, height);
}

function drawDemo(canvas: HTMLCanvasElement, variant: VisionVariant, time: number): void {
  const ctx = canvas.getContext("2d");
  if (ctx === null) {
    return;
  }
  const width = canvas.width;
  const height = canvas.height;
  ctx.fillStyle = "#f4f4f5";
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = "#d8d8d8";
  ctx.fillRect(0, height - height * 0.28, width, height * 0.28);
  ctx.strokeStyle = "#888888";
  ctx.lineWidth = 1;
  for (let x = 0; x <= width; x += width / 8) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += height / 8) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  const cx1 = width * (0.32 + 0.03 * Math.sin(time / 900));
  const cy1 = height * 0.36 + height * 0.02 * Math.cos(time / 700);
  const cx2 = width * (0.68 + 0.03 * Math.cos(time / 1100));
  const cy2 = height * 0.36 + height * 0.02 * Math.sin(time / 800);

  ctx.fillStyle = "#e86875";
  ctx.fillRect(cx1 - 18, cy1 - 18, 36, 36);
  ctx.strokeStyle = "#9f3344";
  ctx.strokeRect(cx1 - 18, cy1 - 18, 36, 36);
  ctx.fillStyle = "#3b5aef";
  ctx.fillRect(cx2 - 18, cy2 - 18, 36, 36);
  ctx.strokeStyle = "#1f3a9f";
  ctx.strokeRect(cx2 - 18, cy2 - 18, 36, 36);

  if (variant === "detection") {
    ctx.strokeStyle = "#e86875";
    ctx.lineWidth = 2;
    ctx.strokeRect(cx1 - 26, cy1 - 26, 52, 52);
    ctx.fillStyle = "#e86875";
    ctx.font = `${Math.max(11, Math.round(height * 0.04))}px ${FONT_FAMILY}`;
    ctx.fillText(`红色物块 96%`, cx1 - 24, cy1 - 32);
    ctx.strokeStyle = "#3b5aef";
    ctx.strokeRect(cx2 - 26, cy2 - 26, 52, 52);
    ctx.fillStyle = "#3b5aef";
    ctx.fillText(`蓝色物块 94%`, cx2 - 24, cy2 - 32);
  }

  ctx.fillStyle = "#191f1a";
  ctx.font = `700 ${Math.max(12, Math.round(height * 0.045))}px ${FONT_FAMILY}`;
  ctx.fillText(variant === "raw" ? "相机演示" : "资源识别", 12, height * 0.12);
  ctx.fillStyle = "#666666";
  ctx.font = `${Math.max(10, Math.round(height * 0.032))}px ${FONT_FAMILY}`;
  ctx.fillText("帧率：30 FPS    相机：正常", 12, height - 12);
}

function VisionImage({
  context,
}: {
  context: PanelExtensionContext;
}): ReactElement {
  const [config, setConfig] = useState<VisionImageConfig>(() =>
    asConfig(context.initialState),
  );
  const [liveMessage, setLiveMessage] = useState<unknown>();
  const [lastLiveAt, setLastLiveAt] = useState<number>(0);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [, setTick] = useState(0);

  useLayoutEffect(() => {
    context.watch("currentFrame");
    context.watch("topics");
    context.onRender = (renderState: Immutable<RenderState>, done: () => void) => {
      setRenderDone(() => done);
      const frame = renderState.currentFrame ?? [];
      let latest: MessageEvent | undefined;
      for (const event of frame) {
        if (event.topic === config.topic) {
          latest = event;
        }
      }
      if (latest !== undefined) {
        setLiveMessage(latest.message);
        setLastLiveAt(Date.now());
      }
    };
    context.subscribe([{ topic: config.topic }]);
    context.setDefaultPanelTitle(config.title);
  }, [config, context]);

  useEffect(() => {
    const timer = setInterval(() => setTick((value) => value + 1), 1000);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    renderDone?.();
  }, [renderDone, liveMessage]);

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
              label: config.variant === "raw" ? "相机图像话题" : "识别结果话题",
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
  const demoMode = effectiveMode === "DEMO";

  useEffect(() => {
    const canvas = canvasRef.current;
    if (canvas === null || canvas.width === 0 || canvas.height === 0) {
      return;
    }
    if (demoMode) {
      drawDemo(canvas, config.variant, Date.now());
      return;
    }
    if (liveMessage !== undefined) {
      const imageData = imageDataOf(liveMessage);
      if (imageData !== undefined) {
        canvas.width = imageData.width;
        canvas.height = imageData.height;
        canvas.getContext("2d")?.putImageData(imageData, 0, 0);
      }
    }
  }, [demoMode, liveMessage, config.variant]);

  const placeholder =
    config.mode === "LIVE" || (config.mode === "AUTO" && !demoMode)
      ? config.placeholder
      : config.placeholder;

  return (
    <div
      style={{
        alignItems: "center",
        background: config.background,
        boxSizing: "border-box",
        display: "flex",
        fontFamily: FONT_FAMILY,
        height: "100%",
        justifyContent: "center",
        margin: 0,
        overflow: "hidden",
        padding: 0,
        width: "100%",
      }}
    >
      {demoMode ? (
        <canvas
          ref={canvasRef}
          style={{ height: "100%", objectFit: "contain", width: "100%" }}
          width={640}
          height={360}
        />
      ) : liveMessage !== undefined ? (
        <canvas
          ref={canvasRef}
          style={{ height: "100%", objectFit: "contain", width: "100%" }}
        />
      ) : (
        <span style={{ color: "#888888", fontSize: 13 }}>{placeholder}</span>
      )}
    </div>
  );
}

export function initVisionImage(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<VisionImage context={context} />);
  return () => {
    root.unmount();
  };
}