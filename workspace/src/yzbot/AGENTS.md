# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Humble workspace source tree. `mybot_description/`
contains URDF/Xacro models, Gazebo worlds, and robot assets. `mybot/` provides
MoveIt configuration, launch files, and pick/place executables.
`bot_navigation/` contains Nav2 maps, parameters, zone configuration, and
navigation launches. `tools_demo/` implements task parsing, perception,
mission coordination, navigation helpers, acceptance checks, and the web
dashboard. `IFRA_LinkAttacher/` is the Gazebo attachment plugin and its message
package. Python tests live in each package's `test/` directory; supporting
design and handoff material belongs in `docs/`.

## Build, Test, and Development Commands

Run commands from the workspace root containing `src/yzbot`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

For fast behavioral feedback, run:

```bash
python3 -m pytest -q src/yzbot/mybot/test src/yzbot/tools_demo/test
ros2 launch tools_demo competition_system.launch.py
```

Use `--packages-select mybot tools_demo` while iterating on a limited change.
Do not commit generated `build/`, `install/`, `log/`, cache, or runtime files.

## Coding Style & Naming Conventions

Python uses four-space indentation, PEP 8 naming, concise docstrings, and type
hints for public helpers. Use `snake_case` for functions, topics, parameters,
and modules; use `PascalCase` for classes and ROS message types. Keep ROS topic
names absolute and grouped under `/competition/`. C++ changes should follow
ROS 2 conventions and the existing plugin style. The test suite enforces
`ament_flake8`, `ament_pep257`, copyright checks, and XML/package validation.

## Testing Guidelines

Name Python tests `test_*.py` and test functions `test_<behavior>()`. Add unit
tests for parsing and math, contract tests for configuration or launch
invariants, and a Gazebo run for navigation or manipulation changes. Preserve
safety-window assertions; never make a failing physical test pass solely by
loosening tolerances. Record the task, terminal mission state, elapsed time,
and Gazebo entity truth for acceptance runs.

## Commit & Pull Request Guidelines

Follow the existing concise, imperative history, for example
`competition: harden repeated pickup delivery` or `step-11: add ...`. Keep
commits scoped and exclude unrelated formatting changes. Pull requests should
describe the observed problem, the fix, commands and results used for
verification, affected ROS packages, and remaining risks. Include dashboard
screenshots or relevant log excerpts when behavior is visible or simulation
dependent. Never commit credentials, private keys, model files, or machine-
specific absolute paths.

## LLM Runtime Defaults

The competition parser only **verifies** a deterministic candidate plan, so
Qwen thinking mode must stay **off** (`enable_thinking` / `llm_enable_thinking`
= `false` in `tools_demo/config/task_parser.yaml` and `task_parser_node`
defaults). Thinking inflated verify latency from ~2 s to ~20 s without changing
the accepted plan. Keep the default `qwen3.8-max` unless the user names a
different model. Do not enable thinking for the verify/plan-publish path; only
turn it on for open-ended solving if that feature is explicitly requested.
