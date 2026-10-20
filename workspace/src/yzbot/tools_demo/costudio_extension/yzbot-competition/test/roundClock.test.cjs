const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");
const code = ts.transpileModule(fs.readFileSync(require.resolve("../src/roundClock.ts"), "utf8"), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
}).outputText;
const key = "yzbot.competition.roundClock.v2";
function context(data = new Map()) {
  const exports = {};
  vm.runInNewContext(code, { exports, window: {
    localStorage: { getItem: k => data.get(k) ?? null,
      setItem: (k, v) => data.set(k, v), removeItem: k => data.delete(k) },
    addEventListener() {}, removeEventListener() {},
  }});
  return exports;
}
const start = 100000;
function record(c) { c.markRoundStart("click-123", start); return c.readRoundClock(); }
function status(state, extra = {}) {
  return { mission_task_id: "plan-456", task_manager_state: state,
    parse_result: { request_id: "click-123", task_id: "plan-456",
      state: "published", published: true }, ...extra };
}
test("no click: live/demo/elapsed status cannot start the clock", () => {
  const c = context();
  assert.equal(c.advanceRoundClock(undefined, status("navigating"), start + 2000), undefined);
});
test("click starts immediately and idle/parsing never stop it", () => {
  const c = context(); const r = record(c);
  assert.equal(r.startedAt, start); assert.equal(r.phase, "pending");
  assert.equal(c.advanceRoundClock(r, { task_running: false, current_state: "idle" }, start + 4000), r);
});
test("old start-only anchor is ignored after upgrade", () => {
  const c = context(new Map([["yzbot.competition.roundStart", '{"taskId":"old","startedAt":1}']]));
  assert.equal(c.readRoundClock(), undefined);
});
test("old completed mission while new parse is published does not stop new click", () => {
  const c = context(); const r = record(c);
  const next = c.advanceRoundClock(r, status("completed", {
    mission_task_id: "old-plan", mission_finished_at_unix_ms: start - 1000 }), start + 5000);
  assert.equal(next.stoppedAt, undefined); assert.equal(next.phase, "pending");
});
test("explicit request/plan alias binds the actual executing mission", () => {
  const c = context(); const next = c.advanceRoundClock(record(c), status("navigating"), start + 5000);
  assert.equal(next.missionTaskId, "plan-456"); assert.equal(next.phase, "running");
  assert.equal(next.startedAt, start); // includes transmission and parsing
});
test("substring and punctuation-normalized IDs are never accepted", () => {
  const c = context(); const r = record(c);
  for (const id of ["click123", "prefix-click-123", "click-12"]) {
    assert.equal(c.advanceRoundClock(r, status("completed", { parse_result: {},
      mission_task_id: id, mission_finished_at_unix_ms: start + 3000 }), start + 5000), r);
  }
});
test("late reconnect freezes at backend endpoint, not receipt time", () => {
  const c = context(); const next = c.advanceRoundClock(record(c), status("completed", {
    mission_finished_at_unix_ms: start + 240123 }), start + 500000);
  assert.equal(next.stoppedAt - next.startedAt, 240123); assert.equal(next.phase, "completed");
});
test("unrelated status, repeated finals and elapsed uptime cannot change frozen clock", () => {
  const c = context(); const r = c.advanceRoundClock(record(c), status("completed", {
    mission_finished_at_unix_ms: start + 2000 }), start + 4000);
  assert.equal(c.advanceRoundClock(r, status("navigating", { elapsed_sec: 999999 }), start + 500000), r);
});
test("missing or invalid endpoint still freezes at the local terminal observation", () => {
  const c = context(); const r = record(c);
  for (const time of [undefined, null, NaN]) {
    const next = c.advanceRoundClock(r, status("completed", { mission_finished_at_unix_ms: time }), start + 3000);
    assert.equal(next.stoppedAt, start + 3000);
    assert.equal(next.phase, "completed");
  }
});
test("previous-round or behind-clock endpoint falls back to the local observation", () => {
  const c = context(); const r = record(c);
  const next = c.advanceRoundClock(r, status("completed", { mission_finished_at_unix_ms: start - 1 }), start + 3000);
  assert.equal(next.stoppedAt, start + 3000);
  assert.equal(next.phase, "completed");
});
test("future endpoint from an ahead backend clock is clamped, not rejected", () => {
  const c = context(); const r = record(c);
  const next = c.advanceRoundClock(r, status("completed", { mission_finished_at_unix_ms: start + 100000 }), start + 3000);
  assert.equal(next.stoppedAt, start + 3000);
  assert.equal(next.phase, "completed");
});
test("terminal frame without an endpoint freezes exactly once", () => {
  const c = context(); const r = record(c);
  const frozen = c.advanceRoundClock(r, status("failed"), start + 3000);
  assert.equal(frozen.stoppedAt, start + 3000);
  assert.equal(frozen.phase, "failed");
  assert.equal(c.advanceRoundClock(frozen, status("failed"), start + 9000), frozen);
});
test("parse failure only ends matching submission", () => {
  const c = context(); const r = record(c);
  assert.equal(c.advanceRoundClock(r, { parse_result: { request_id: "old", state: "failed" } }), r);
  const failed = c.advanceRoundClock(r, { parse_result: { request_id: r.taskId, state: "failed" } }, start + 2000);
  assert.equal(failed.phase, "failed"); assert.equal(failed.stoppedAt, start + 2000);
});
test("failed/canceled/aborted missions freeze with distinct non-success phase", () => {
  const c = context();
  for (const state of ["failed", "canceled", "aborted"]) {
    const next = c.advanceRoundClock(record(c), status(state, { mission_finished_at_unix_ms: start + 3000 }), start + 5000);
    assert.equal(next.stoppedAt, start + 3000); assert.notEqual(next.phase, "completed");
  }
});
test("separate panel contexts and refresh retain the exact start and stop", () => {
  const data = new Map(); const c = context(data); record(c);
  c.observeRoundStatus(status("completed", { mission_finished_at_unix_ms: start + 20123 }));
  const refreshed = context(data).readRoundClock();
  assert.equal(refreshed.startedAt, start); assert.equal(refreshed.stoppedAt, start + 20123);
});
test("new click alone resets a finished round, and old final stays inert", () => {
  const c = context(); record(c); c.finishRoundSubmission("click-123", start + 1000);
  c.markRoundStart("new-click", start + 5000);
  c.observeRoundStatus(status("completed", { mission_finished_at_unix_ms: start + 2000 }));
  assert.equal(c.readRoundClock().startedAt, start + 5000); assert.equal(c.readRoundClock().stoppedAt, undefined);
});
test("corrupt storage is rejected; explicit clear propagates to another context", () => {
  const data = new Map(); const c = context(data); record(c);
  data.set(key, "broken"); assert.equal(c.readRoundClock(), undefined);
  record(c); const other = context(data); other.clearRoundStart();
  assert.equal(c.readRoundClock(), undefined);
});
test("wrong submission failure cannot stop current click", () => {
  const c = context(); record(c); c.finishRoundSubmission("old", start + 5000);
  assert.equal(c.readRoundClock().stoppedAt, undefined);
});
