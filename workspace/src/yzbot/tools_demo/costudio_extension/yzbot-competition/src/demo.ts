import { CompetitionState, DEFAULT_DEMO_STATE } from "./state";

type Listener = (state: CompetitionState) => void;

interface DemoStep {
  at: number;
  action: string;
  status: string;
  red?: number;
  blue?: number;
  index?: number;
  completed?: number;
  running?: boolean;
}

const STEPS: DemoStep[] = [
  { at: 0, action: "等待任务", status: "机器人待命", index: 1, completed: 0, running: false },
  { at: 2, action: "正在解析任务", status: "任务运行中", running: true },
  { at: 4, action: "前往资源点", status: "正在导航" },
  { at: 7, action: "正在避障", status: "正在避障" },
  { at: 10, action: "正在识别物块", status: "发现红色物块" },
  { at: 12, action: "机械臂规划中", status: "机械臂规划中" },
  { at: 14, action: "正在抓取", status: "机械臂运动中" },
  { at: 16, action: "夹取完成", status: "已夹取" },
  { at: 18, action: "前往C区", status: "正在运输" },
  { at: 21, action: "机械臂规划中", status: "机械臂规划中" },
  { at: 23, action: "正在放置", status: "正在放置" },
  { at: 25, action: "已释放", status: "已释放", red: 2, index: 2, completed: 1 },
  { at: 28, action: "正在执行下一任务", status: "任务运行中" },
];

const CYCLE_SECONDS = 30;

class DemoEngine {
  private state: CompetitionState = { ...DEFAULT_DEMO_STATE };
  private listeners = new Set<Listener>();
  private timer: ReturnType<typeof setInterval> | undefined;
  private seconds = 0;

  subscribe(listener: Listener): () => void {
    this.listeners.add(listener);
    listener({ ...this.state });
    return () => {
      this.listeners.delete(listener);
    };
  }

  start(): void {
    if (this.timer !== undefined) {
      return;
    }
    this.timer = setInterval(() => {
      this.tick();
    }, 1000);
  }

  stop(): void {
    if (this.timer !== undefined) {
      clearInterval(this.timer);
      this.timer = undefined;
    }
    this.seconds = 0;
    this.state = { ...DEFAULT_DEMO_STATE };
    this.emit();
  }

  isRunning(): boolean {
    return this.timer !== undefined;
  }

  private tick(): void {
    this.seconds = (this.seconds + 1) % CYCLE_SECONDS;
    const base = { ...DEFAULT_DEMO_STATE };
    let chosen: DemoStep = STEPS[0]!;
    for (const step of STEPS) {
      if (this.seconds >= step.at) {
        chosen = step;
      }
    }
    const next: CompetitionState = {
      ...base,
      currentAction: chosen.action,
      robotStatus: chosen.status,
      currentTaskIndex: chosen.index ?? base.currentTaskIndex,
      redCount: chosen.red ?? base.redCount,
      completedCount: chosen.completed ?? base.completedCount,
      taskRunning: chosen.running ?? true,
      rawImageAvailable: true,
      detectionImageAvailable: true,
    };
    this.state = next;
    this.emit();
  }

  private emit(): void {
    const snapshot = { ...this.state };
    for (const listener of this.listeners) {
      listener(snapshot);
    }
  }
}

export const demoEngine = new DemoEngine();