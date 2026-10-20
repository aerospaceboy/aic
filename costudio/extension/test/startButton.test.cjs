const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");
const code = ts.transpileModule(fs.readFileSync(require.resolve("../src/startButton.ts"), "utf8"), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
}).outputText;
function context() {
  const exports = {};
  vm.runInNewContext(code, { exports });
  return exports;
}
// The demo fallback deliberately looks like a running competition.
const DEMO_FALLBACK = { taskRunning: true, currentAction: "前往资源点" };
const IDLE_LIVE = { taskRunning: false, currentAction: "等待指令" };
const FINISHED_LIVE = { taskRunning: false, currentAction: "任务完成" };

test("AUTO-resolved DEMO is idle even though demo telemetry says running", () => {
  const view = context().startButtonView("DEMO", DEMO_FALLBACK, false, true);
  assert.equal(view.running, false);
  assert.equal(view.finished, false);
  assert.equal(view.disabled, false);
  assert.equal(view.label, "开始演示");
});
test("running demo offers a stop action without requiring a publisher", () => {
  const view = context().startButtonView("DEMO", DEMO_FALLBACK, true, false);
  assert.equal(view.running, true);
  assert.equal(view.disabled, false);
  assert.equal(view.label, "停止演示");
});
test("live running task disables starting another round", () => {
  const view = context().startButtonView("LIVE", { taskRunning: true, currentAction: "正在抓取" }, false, true);
  assert.equal(view.running, true);
  assert.equal(view.disabled, true);
  assert.equal(view.label, "任务运行中");
});
test("live finished task offers a restart instead of staying disabled", () => {
  const view = context().startButtonView("LIVE", FINISHED_LIVE, false, true);
  assert.equal(view.running, false);
  assert.equal(view.finished, true);
  assert.equal(view.disabled, false);
  assert.equal(view.label, "重新开始");
});
test("live idle task is ready when the connection can publish", () => {
  const view = context().startButtonView("LIVE", IDLE_LIVE, false, true);
  assert.equal(view.disabled, false);
  assert.equal(view.label, "开始任务");
});
test("live panel without a publisher is visibly disabled", () => {
  const view = context().startButtonView("LIVE", undefined, false, false);
  assert.equal(view.running, false);
  assert.equal(view.disabled, true);
  assert.equal(view.label, "开始任务");
});
