# bicycle_navigation

SLAM and Nav2 for the bicycle's forward-only, Ackermann-type kinematics.

- `config/slam.yaml`: slam_toolbox, shared by mapping and localization.
- `config/nav2.yaml`: Smac Hybrid-A*, Regulated Pure Pursuit, costmaps with the bicycle's
  footprint, velocity smoother, collision monitor, behavior server, BT navigator.
- `behavior_trees/`: Jazzy replanning/recovery trees without Spin and BackUp.
- `maps/`: maps of the `bicycle_gazebo` worlds (see `maps/README.md`).
- `launch/slam.launch.py`: `mode:=mapping|localization`, `map:=<path without extension>`,
  `use_sim_time`.
- `launch/navigation.launch.py`: the Nav2 servers and their lifecycle manager.
  Arguments: `use_sim_time`, `params_file`, `autostart`, `log_level`.

See [docs/navigation.md](../docs/navigation.md) and
[docs/mapping_and_localization.md](../docs/mapping_and_localization.md).
