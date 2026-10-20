# bicycle_bringup

Composition of the simulated system, RViz configurations, teleop and cross-package tests.

- `launch/sim_bringup.launch.py`: description, Gazebo, controllers and optionally SLAM,
  Nav2 and RViz. Own arguments: `slam` (`true`), `nav` (`false`), `rviz` (`true`); the
  component launch files' arguments (`world`, `headless`, `x`, `y`, `yaw`, `mode`, `map`,
  ...) pass through.
- `rviz/slam.rviz`, `rviz/navigation.rviz`.
- `ros2 run bicycle_bringup teleop_keyboard`: keyboard teleop publishing sim-time
  `TwistStamped` on `/cmd_vel_teleop` (0.4 m/s, 0.4 rad/s by default).
- `test/test_consistency.py`: values repeated from `geometry.yaml` and the frame/topic
  contract across packages.
- `test/test_launch_files.py`: every launch file loads and declares its arguments.
- `test/integration/test_sim_integration.py`: headless Gazebo end-to-end test
  (CTest label `sim_integration`).
