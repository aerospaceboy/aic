/** Pure start-button presentation logic, independent of React and ROS. */
import type { CompetitionState, DataMode } from "./state";

export type StartButtonView = {
  /** True while a round/demo is running, so the button must not start a new one. */
  running: boolean;
  /** True when the last live mission reached its terminal success state. */
  finished: boolean;
  /** True when clicking cannot do anything useful in the current mode. */
  disabled: boolean;
  label: string;
};

/**
 * Resolve the button state for the already AUTO-resolved `effectiveMode`.
 *
 * The demo fallback state intentionally looks like a competition in progress
 * (`DEFAULT_DEMO_STATE.taskRunning === true`) so the cards show plausible
 * values before ROS connects. That must never leak into the button: in DEMO
 * mode only the demo engine can be running, otherwise the button would stay
 * "任务运行中" and disabled forever in AUTO with no live frames.
 */
export function startButtonView(
  effectiveMode: DataMode,
  state: CompetitionState | undefined,
  demoRunning: boolean,
  canPublish: boolean,
): StartButtonView {
  if (effectiveMode === "DEMO") {
    return {
      running: demoRunning,
      finished: false,
      // Demo playback works without a live ROS data source by design.
      disabled: false,
      label: demoRunning ? "停止演示" : "开始演示",
    };
  }
  const running = state?.taskRunning === true;
  const finished = !running && state?.currentAction === "任务完成";
  const disabled = running || !canPublish;
  return {
    running,
    finished,
    disabled,
    label: running ? "任务运行中" : finished ? "重新开始" : "开始任务",
  };
}
