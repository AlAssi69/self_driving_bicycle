# bicycle_gazebo

Gazebo Harmonic simulation of the bicycle.

- `worlds/obstacles_course.sdf`: walled 18.75 m x 7.75 m arena with a cone 6 m ahead of the
  start pose.
- `models/`: `box` (the arena), `room` (unused floor plan for larger worlds) and a vendored
  CC0 `construction_cone` from Gazebo Fuel, so nothing is downloaded at run time.
- `config/gz_bridge.yaml`: `/clock` and `/scan` from Gazebo to ROS.
- `hooks/`: adds `models/` and `worlds/` to `GZ_SIM_RESOURCE_PATH` when the workspace is
  sourced.
- `launch/sim.launch.py`: starts Gazebo, spawns the robot from `/robot_description` and
  starts the bridge. Arguments: `world` (`obstacles_course.sdf`), `headless` (`false`),
  `x`, `y`, `yaw` (spawn pose).

robot_state_publisher must publish `/robot_description` with `use_sim:=true` (see
`bicycle_bringup/launch/sim_bringup.launch.py`).
