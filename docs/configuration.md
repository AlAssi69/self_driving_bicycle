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
| `lidar.min_range`, `max_range`, `update_rate` | `bicycle_hardware/config/ydlidar.yaml` | equal (the simulation models the real lidar) |
| `steering.max_angle`, `max_velocity` | `bicycle_hardware/firmware/bicycle_uno/config.hpp` (`kHardLimits`, in steps) | ≥ the limit × steps per radian from `hardware.yaml` |
| `traction.max_velocity` | `bicycle_hardware/config/hardware.yaml` (`drive_max_pwm`) | the PWM law reaches it |

The same test checks the shared names: robot frame `base_footprint`, `odom`, `map`, the
`/scan` topic in the URDF, the bridge, slam_toolbox, the costmaps and the collision monitor,
and `laser_frame` in the lidar driver. It also checks that the hardware parameters reach the
plugin and that their baud rate matches the firmware's `protocol.hpp`.

## Changing the robot

1. Edit `geometry.yaml`.
2. Update the repeated values listed above.
3. Run `colcon build && colcon test --ctest-args -LE sim_integration`. A failing
   `test_consistency` names the value that no longer agrees.

The real bicycle's calibration (encoder, stepper, drive) is in
`bicycle_hardware/config/hardware.yaml`; [hardware.md](hardware.md#calibration) explains each
value.

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
| `bicycle_hardware/config/hardware.yaml` | real robot: serial link, encoder, stepper and drive calibration |
| `bicycle_hardware/config/ydlidar.yaml` | real robot: YDLIDAR X4 Pro driver |
| `bicycle_hardware/firmware/bicycle_uno/config.hpp` | real robot: Arduino wiring and hard limits |

`use_sim_time` is never set in these files: launch files pass it, and a YAML value would
override the launch value.

## Using the description with other hardware

```bash
xacro bicycle.urdf.xacro hardware_plugin:=<your_package>/<YourSystem> \
  hardware_params_file:=<your_params>.yaml
```

Without `use_sim:=true` no Gazebo plugin is added. Each key of the (flat) parameter file
becomes a `<param>` of the plugin. The hardware layer then runs `ros2_control_node` with
`bicycle_controller.yaml`, as `bicycle_hardware/launch/hardware.launch.py` does.
