# Configuration

## Single source of truth

Every physical parameter of the bicycle lives in
[`bicycle_description/config/geometry.yaml`](../bicycle_description/config/geometry.yaml)
(SI units). The URDF reads it directly through `xacro.load_yaml()`.

The controller and Nav2 read plain ROS parameter files, which cannot import other files.
The few values they repeat are therefore literal, and a test checks each one against
`geometry.yaml` instead of generating YAML at build time:

| Value | Repeated in | Rule checked by `bicycle_bringup/test/test_consistency.py` |
|---|---|---|
| `wheelbase` | `bicycle_control/config/bicycle_controller.yaml` | equal |
| `wheel.radius` | `bicycle_controller.yaml` (`traction_wheel_radius`) | equal |
| `wheelbase`, `steering.max_angle` | `nav2.yaml`: Smac `minimum_turning_radius` | ≥ wheelbase / tan(max_angle) |
| same | `nav2.yaml`: RPP `min_lookahead_dist` | ≥ minimum turning radius |
| `traction.max_velocity`, `wheel.radius` | `nav2.yaml`: velocity smoother, RPP speed | ≤ wheel speed limit × radius |
| turning radius | `nav2.yaml`: velocity smoother max yaw rate | ≤ max speed / turning radius |
| `wheelbase`, `wheel.radius`, `wheel.width` | `nav2.yaml`: costmap `footprint` | covers both wheels |
| footprint | `nav2.yaml`: global `inflation_radius` | ≥ footprint circumscribed radius |
| `lidar.max_range` | `bicycle_navigation/config/slam.yaml` (`max_laser_range`) | ≤ |

The same test checks the shared names: robot frame `base_footprint`, `odom`, `map`, and the
`/scan` topic in the URDF, the bridge, slam_toolbox, the costmaps and the collision monitor.

## Changing the robot

1. Edit `geometry.yaml`.
2. Update the repeated values listed above.
3. Run `colcon build && colcon test --ctest-args -LE sim_integration`. A failing
   `test_consistency` names the value that no longer agrees.

## Where the other settings live

| File | Purpose |
|---|---|
| `bicycle_control/config/bicycle_controller.yaml` | controller_manager rate, bicycle controller (frames, timeout, odometry) |
| `bicycle_control/config/twist_mux_topics.yaml` | velocity sources and priorities |
| `bicycle_control/config/twist_mux_locks.yaml` | e-stop lock |
| `bicycle_gazebo/config/gz_bridge.yaml` | topics bridged from Gazebo (`/clock`, `/scan`) |
| `bicycle_navigation/config/slam.yaml` | slam_toolbox, shared by mapping and localization |
| `bicycle_navigation/config/nav2.yaml` | Nav2 servers and costmaps |
| `bicycle_navigation/behavior_trees/*.xml` | Nav2 behavior trees |

`use_sim_time` is never set in these files: launch files pass it, and a YAML value would
override the launch value.

## Using the description on real hardware

```bash
xacro bicycle.urdf.xacro hardware_plugin:=<your_package>/<YourSystem>
```

Without `use_sim:=true` no Gazebo plugin is added. The hardware package then runs
`ros2_control_node` with `bicycle_controller.yaml` and starts `control.launch.py`.
