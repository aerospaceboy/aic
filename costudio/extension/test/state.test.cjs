const {test} = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");
const exportsObject = {};
const code = ts.transpileModule(fs.readFileSync(require.resolve("../src/state.ts"), "utf8"), {
  compilerOptions: {module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020},
}).outputText;
vm.runInNewContext(code, {exports: exportsObject});
const {parseCompetitionState, DEFAULT_DEMO_STATE} = exportsObject;
test("current task uses actual executing color's target, not the first plan color", () => {
  const state = parseCompetitionState({data: JSON.stringify({
    current_action: "navigating_destination", current_state: "navigating_destination",
    current_target_zone: "C", target_zones: {red: "A", blue: "C"}, task_running: true,
  })}, DEFAULT_DEMO_STATE);
  assert.equal(state.currentAction, "前往C区");
});
test("completed mission shows completion instead of idle fallback", () => {
  const state = parseCompetitionState({data: JSON.stringify({
    current_action: "completed", current_state: "completed", task_running: false,
  })}, DEFAULT_DEMO_STATE);
  assert.equal(state.currentAction, "任务完成");
  assert.equal(state.robotStatus, "任务完成");
  assert.equal(state.taskRunning, false);
});
