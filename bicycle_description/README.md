# bicycle_description

URDF/xacro model of the self-driving bicycle, independent of the simulator and the hardware.

- `config/geometry.yaml`: every dimension, mass and limit (single source of truth).
- `urdf/bicycle.urdf.xacro`: the robot. Arguments:
  - `use_sim` (`false`): add the gz_ros2_control Gazebo plugin.
  - `hardware_plugin` (empty): ros2_control system plugin; empty selects
    `gz_ros2_control/GazeboSimSystem` with `use_sim`, otherwise `mock_components/GenericSystem`.
  - `controllers_file`: controller parameters for gz_ros2_control (`use_sim` only).
- `launch/description.launch.py`: robot_state_publisher (`use_sim_time`, plus the xacro
  arguments above).
- `launch/display.launch.py`: the model in RViz with joint sliders, no simulator.

Frames: `base_footprint` (ground under the rear axle, root) → `base_link` (rear axle) →
`chassis`, `rear_wheel`, `steering_link` → `front_wheel`, `laser_frame`.
See [docs/architecture.md](../docs/architecture.md).
