const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");
const exportsObject = {};
const code = ts.transpileModule(fs.readFileSync(require.resolve("../src/format.ts"), "utf8"), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
}).outputText;
vm.runInNewContext(code, { exports: exportsObject });
const {
  getPath, formatPathValue, currentActionLabel, statusLabel,
} = exportsObject;

test("getPath reads nested objects and array indexes", () => {
  const message = { resources: { red: 3 }, items: [{ zone: "A" }, { zone: "C" }] };
  assert.equal(getPath(message, "resources.red"), 3);
  assert.equal(getPath(message, "items.1.zone"), "C");
  assert.equal(getPath(message, "items.9.zone"), undefined);
  assert.equal(getPath(message, "resources.blue"), undefined);
  assert.equal(getPath(message, ""), undefined);
  assert.equal(getPath(message, undefined), undefined);
});

test("formatPathValue applies prefix/suffix and JSON-encodes objects", () => {
  assert.equal(formatPathValue({ a: 1 }, "a", "x", "y", "?"), "x1y");
  assert.equal(formatPathValue({ a: null }, "a", undefined, undefined, "?"), "?");
  assert.equal(formatPathValue({}, "a", undefined, undefined, "?"), "?");
  assert.equal(formatPathValue({ a: { b: 2 } }, "a", "=", "", "?"), '={"b":2}');
});

test("currentActionLabel prefers the actual delivery zone", () => {
  assert.equal(currentActionLabel({
    current_state: "navigating_destination", current_target_zone: "c",
  }, "fallback"), "前往C区");
  assert.equal(currentActionLabel({ current_state: "completed" }, "fallback"), "任务完成");
  assert.equal(currentActionLabel({ current_state: "unknown-state" }, "fallback"), "fallback");
});

test("statusLabel maps grasp and task states without inventing success", () => {
  assert.equal(statusLabel({ current_state: "completed" }, "?"), "任务完成");
  assert.equal(statusLabel({ current_state: "failed" }, "?"), "任务失败");
  assert.equal(statusLabel({ grasp_state: "attached" }, "?"), "夹持中");
  assert.equal(statusLabel({ grasp_state: "released" }, "?"), "已释放");
  assert.equal(statusLabel({ current_state: "picking" }, "?"), "抓取中");
  assert.equal(statusLabel({ current_state: "navigating_pickup" }, "?"), "导航中");
  assert.equal(statusLabel({}, "?"), "?");
});
