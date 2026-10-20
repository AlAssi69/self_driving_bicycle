# bicycle_bringup

Composition of the simulated and the real system, RViz configurations, teleop and
cross-package tests.

- `launch/sim_bringup.launch.py`: description, Gazebo, controllers and optionally SLAM,
  Nav2 and RViz. Own arguments: `slam` (`true`), `nav` (`false`), `rviz` (`true`); the
  component launch files' arguments (`world`, `headless`, `x`, `y`, `yaw`, `mode`, `map`,
  ...) pass through.
- `launch/robot_bringup.launch.py`: the real bicycle, with `bicycle_hardware` in place of
  Gazebo. Own arguments: `slam` (`true`), `nav` (`false`), `hardware_params_file`
  (`bicycle_hardware/config/hardware.yaml`), `device` (`/dev/bicycle_arduino`); `lidar`,
  `mode`, `map`, ... pass through.
- `launch/rviz.launch.py`: RViz with `config:=slam|navigation`, `use_sim_time`.
- `rviz/slam.rviz`, `rviz/navigation.rviz`.
- `ros2 run bicycle_bringup teleop_keyboard [--real]`: keyboard teleop publishing
  `TwistStamped` on `/cmd_vel_teleop` (0.4 m/s, 0.4 rad/s by default), stamped with sim
  time, or wall time with `--real`.
- `test/test_consistency.py`: values repeated from `geometry.yaml`, the frame/topic contract
  across packages, and the real robot's hardware, firmware and lidar settings.
- `test/test_launch_files.py`: every launch file loads and declares its arguments.
- `test/integration/test_robot_integration.py`: the real-robot stack against the fake
  Arduino (`fake_mcu`), no GPU needed.
- `test/integration/test_sim_integration.py`: headless Gazebo end-to-end test
  (CTest label `sim_integration`).
