# Competition dashboard

The dashboard is a read-only scoring view for the ROS2 competition runtime.
It shows the generated question, parsed order and quantities, current mission
progress, navigation status, gripper state, moving obstacles, and the live
2026 v2 navigation view.

The navigation canvas uses the v2 operational bounds and renders four live
sources:

- `trail`: the robot's measured TF trajectory, sampled every 0.04 m;
- `navigation.command_path`: the remaining route for direct precision docking;
- `path`: the Nav2 `/plan` global path when Nav2 is active;
- `local_path`: the `/local_plan` avoidance trajectory, hidden after two
  seconds without a fresh controller update.

A new mission clears the old paths and measured trail. The TF pose is
authoritative; `/amcl_pose` is used only when TF has been stale for one second.

Start it with the mission runtime:

```bash
ros2 launch tools_demo mission.launch.py \
  generate_question:=true \
  use_llm:=true \
  dashboard:=true
```

Open `http://<virtual-machine-ip>:8080/` from the host computer. For the
provided development virtual machine this is normally:

```text
http://192.168.153.128:8080/
```

The service is read-only: it exposes `/`, `/api/state`, and `/healthz` and
does not provide control endpoints. Set `dashboard:=false` when another
process already owns port 8080.

The upper-right map label shows command/global/local point counts as `C/G/L`.
Short v2 legs intentionally bypass Nav2, so `G0 L0` is normal when `C` and the
green measured trail are updating.
