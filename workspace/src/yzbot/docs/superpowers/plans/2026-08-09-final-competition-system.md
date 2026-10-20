# Final Competition System Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify the final ROS 2 competition system from the 0809 baseline with a controlled-disorder ten-cube layout, exactly two moving obstacles, reliable manipulation, official generator integration, validated LLM parsing, and a working dashboard.

**Architecture:** Keep the existing ROS package boundaries and make `config/final_mission_layout.yaml` the authoritative spatial model. A deterministic generator validates and propagates the layout into Gazebo, Nav2, mission, and dashboard consumers; behavioral fixes are introduced through focused regression tests and then verified in the VM.

**Tech Stack:** ROS 2 Humble, Gazebo Classic 11, Nav2, MoveIt 2, Python 3, pytest, XML/SDF, YAML, HTML/JavaScript, llama.cpp OpenAI-compatible API.

---

### Task 1: Establish the final baseline

**Files:**
- Replace from 0809 snapshot: repository source tree excluding generated caches
- Preserve/Create: `docs/superpowers/specs/2026-08-09-final-competition-system-design.md`
- Preserve/Create: `docs/superpowers/plans/2026-08-09-final-competition-system.md`

- [ ] Copy the 0809 snapshot into the writable final workspace without `build/`, `install/`, `log/`, `.pytest_cache`, or `__pycache__`.
- [ ] Compare file inventories and SHA-256 hashes to prove the baseline copy is exact.
- [ ] Run the available local pure-Python tests; record missing dependencies instead of treating them as code failures.
- [ ] Create the final Git branch after repository metadata is restored or initialized from the configured remote.

### Task 2: Make the layout YAML authoritative

**Files:**
- Modify: `config/final_mission_layout.yaml`
- Modify: `tools_demo/scripts/generate_final_layout.py`
- Modify: `tools_demo/test/test_layout_config.py`
- Modify: `tools_demo/test/test_navigation_contract.py`

- [ ] Add a failing test asserting exactly ten unique cubes, ten matching trays, three zones, and two dynamic obstacles.
- [ ] Add a failing test asserting every cube dock is in the arm reach window and every retreat pose clears all tray collision boxes by more than the Nav2 inflation radius.
- [ ] Add a failing test asserting each obstacle cylinder sweep stays outside walls, trays, pickup docks, and zone placement footprints.
- [ ] Run `python3 -m pytest -q tools_demo/test/test_layout_config.py tools_demo/test/test_navigation_contract.py` and confirm failures identify the existing three-obstacle/world-source behavior.
- [ ] Restructure the generator to load the YAML, validate it, and update named XML model poses plus generated configuration sections deterministically.
- [ ] Define ten fixed, staggered pickup positions in open areas while preserving the existing wall and zone geometry.
- [ ] Define two route-crossing obstacle tracks with bounded speed, period, and radius.
- [ ] Regenerate all consumers and rerun both contract files until green.

### Task 3: Align navigation and safe departures

**Files:**
- Modify: `bot_navigation/config/competition_zones.yaml`
- Modify: `bot_navigation/launch/competition_navigation.launch.py`
- Modify: `tools_demo/tools_demo/navigation_math.py`
- Modify: `tools_demo/tools_demo/task_navigator.py`
- Test: `tools_demo/test/test_navigation_math.py`
- Test: `tools_demo/test/test_navigation_contract.py`

- [ ] Add failing tests for generated per-cube approach sides, straight reverse pickup departure, stopped turn, and Nav2 handoff.
- [ ] Add failing tests that launch/config/dashboard/monitor all expose exactly the two configured obstacle topics.
- [ ] Run the focused tests and confirm the old hard-coded cube slots and three launch nodes fail them.
- [ ] Load cube approaches and departure routes from generated config rather than fixed module constants.
- [ ] Enforce stop-before-turn and zero stale velocity on navigation completion, cancellation, timeout, and handoff.
- [ ] Launch only the two configured moving-obstacle nodes and rerun navigation tests.

### Task 4: Stabilize grasp, carry, and placement

**Files:**
- Modify: `mybot/pick_place_moveit.py`
- Modify: `tools_demo/tools_demo/mission_coordinator.py`
- Modify: `tools_demo/config/mission.yaml`
- Test: `mybot/test/test_pick_place_math.py`
- Test: `tools_demo/test/test_mission_contract.py`
- Test: `tools_demo/test/test_mission_state_machine.py`

- [ ] Add a failing geometry test proving the requested tool pose centers the cube between the fingers after applying the calibrated tool offset.
- [ ] Add a failing state-machine test proving a cube cannot advance to delivery until attachment and carry-window verification both succeed.
- [ ] Add a failing test proving recovery releases safely and rescans instead of dragging or pushing a failed cube.
- [ ] Run the focused tests and record the exact failures.
- [ ] Separate visual target coordinates, tool goal coordinates, and attached-cube relative coordinates with named conversion helpers.
- [ ] Close the gripper at the centered pose, verify attachment, lift vertically at bounded speed, then validate settled relative drift.
- [ ] Place by vertical descent, open, detach, stabilize, lift vertically, and verify Gazebo truth before reporting completion.
- [ ] Remove duplicated or contradictory mission parameters and restore `mission_timeout_sec` to at most 300 seconds after timing validation.
- [ ] Rerun manipulation and mission tests until green.

### Task 5: Integrate the official problem generator and validated LLM parsing

**Files:**
- Modify: `tools_demo/tools_demo/task_question_node.py`
- Modify: `tools_demo/tools_demo/task_parser_node.py`
- Modify: `tools_demo/tools_demo/problem_solver.py`
- Modify: `tools_demo/launch/mission.launch.py`
- Modify: `tools_demo/launch/competition_system.launch.py`
- Modify: `tools_demo/config/task_parser.yaml`
- Test: `tools_demo/test/test_problem_solver.py`
- Test: `tools_demo/test/test_task_protocol.py`
- Test: `tools_demo/test/test_mission_contract.py`

- [ ] Add failing tests for parsing both official wardrobe and pet-shop outputs, binary timeout/nonzero exit, duplicate suppression, and UTF-8 output extraction.
- [ ] Add failing tests that competition mode requires a validated LLM result while deterministic arithmetic independently rejects hallucinated values.
- [ ] Run the focused tests and confirm the current custom direct-question generator fails official-binary expectations.
- [ ] Add an injectable subprocess runner that executes the configured official binary without shell interpolation and publishes the captured problem.
- [ ] Make direct-task generation an explicit development mode rather than the competition default.
- [ ] Route official problems through LLM extraction plus deterministic validation and publish an actionable error on mismatch.
- [ ] Rerun parser tests until green.

### Task 6: Harden the scoring dashboard

**Files:**
- Modify: `tools_demo/tools_demo/competition_dashboard.py`
- Modify: `tools_demo/tools_demo/dashboard_state.py`
- Modify: `tools_demo/web/dashboard.html`
- Modify: `tools_demo/config/dashboard.yaml`
- Test: `tools_demo/test/test_dashboard_state.py`
- Test: `tools_demo/test/test_mission_contract.py`

- [ ] Add failing tests for valid generated layout loading, clear malformed-layout diagnostics, exactly two live obstacles, and problem submission state.
- [ ] Add a failing HTTP smoke test for `/`, `/state.json`, and `/task/problem` using an ephemeral port.
- [ ] Run the dashboard tests and capture the existing failure.
- [ ] Load the generated layout safely, isolate HTTP startup errors, and expose readiness in dashboard state.
- [ ] Render the actual ten cubes, three zones, two obstacle positions, plan source, progress, and terminal state without nested panels or overlapping labels.
- [ ] Rerun dashboard and mission contract tests until green.

### Task 7: Build and verify in the VM

**Files:**
- Update: `tools_demo/scripts/clean_launch.sh`
- Create/Update: `docs/final_acceptance_report.md`

- [ ] Synchronize the final source tree to `/home/ros/dev_ws/src/yzbot` while excluding generated workspace directories.
- [ ] Run `colcon build --symlink-install --packages-select mybot_description mybot bot_navigation tools_demo` and require zero failures.
- [ ] Run `python3 -m pytest -q /home/ros/dev_ws/src/yzbot/mybot/test /home/ros/dev_ws/src/yzbot/tools_demo/test` and require zero failures.
- [ ] Start headless and verify required nodes, topics, services, actions, transforms, LLM status, and dashboard HTTP readiness.
- [ ] Run one red-cube smoke test and verify navigation, centered pick, attachment, delivery truth, and terminal state.
- [ ] Run `Move 2 red cubes to zone B and 1 blue cube to zone C.` and inspect all three entity truths plus collision logs.
- [ ] Run an official generated problem using the configured mapping rules and verify `source=llm` with deterministic agreement.
- [ ] Record measured elapsed time and any residual risk without overstating success.

### Task 8: Complete logs, operator docs, and Git delivery

**Files:**
- Modify: `docs/codex_execution_log.md`
- Modify: `PROJECT_HANDOFF.md`
- Modify: `项目总结.md`
- Modify: parent `工作日志.md`
- Modify: parent `成员B_学习日志.md`
- Modify: parent `指导书.md`
- Create: `日志.md`

- [ ] Record root causes, changed files, exact test commands, outputs, VM run IDs, measured results, and remaining risks.
- [ ] Update the operator guide with clean build, launch, official generator, manual task, dashboard, log extraction, cancellation, and recovery commands.
- [ ] Scan the repository for credentials, private keys, machine-specific secrets, caches, and generated build artifacts.
- [ ] Commit scoped changes on the new final branch.
- [ ] Push the branch to `cty050422/yzbot-224` and verify the remote branch and commit hash.

