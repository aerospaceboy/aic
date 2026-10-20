const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");
const code = ts.transpileModule(fs.readFileSync(require.resolve("../src/chunk.ts"), "utf8"), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
}).outputText;
function context() {
  const exports = {};
  // Browser globals used by the encoder; Node 20 provides the same values.
  vm.runInNewContext(code, { exports, TextEncoder, btoa });
  return exports;
}

test("chunks reassemble to the exact UTF-8 payload", () => {
  const c = context();
  const original = JSON.stringify({
    task_id: "round-1",
    question: "红色物块数量大于等于3去C区；蓝色去A区。".repeat(8),
    mapping_rules: "x代表红色，y代表蓝色",
  });
  const chunks = c.splitProblemChunks("round-1", original);
  assert.ok(chunks.length > 1);
  assert.deepEqual([...chunks.map((chunk) => chunk.i)], [...Array(chunks.length).keys()]);
  assert.ok(chunks.every((chunk) =>
    chunk.v === 1 && chunk.id === "round-1" && chunk.n === chunks.length));
  const joined = chunks.map((chunk) => chunk.d).join("");
  assert.equal(joined, c.encodeUtf8Base64(original));
  assert.equal(Buffer.from(joined, "base64").toString("utf8"), original);
});

test("short payload is one complete chunk", () => {
  const c = context();
  const chunks = c.splitProblemChunks("round-2", "short");
  assert.equal(chunks.length, 1);
  assert.equal(chunks[0].i, 0);
  assert.equal(chunks[0].n, 1);
  assert.equal(chunks[0].d, c.encodeUtf8Base64("short"));
});

test("empty payload still yields one chunk so the backend sees the id", () => {
  const chunks = context().splitProblemChunks("round-3", "");
  assert.equal(chunks.length, 1);
  assert.equal(chunks[0].d, "");
});

test("invalid data length is rejected instead of emitting broken chunks", () => {
  const c = context();
  assert.throws(() => c.splitProblemChunks("round-4", "x", 0));
  assert.throws(() => c.splitProblemChunks("round-4", "x", 1.5));
});
