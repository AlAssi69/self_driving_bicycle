# Migration from Humble (v1.0.0) to Jazzy (v2.0.0)

Version 1.0.0 (tag `v1.0.0-humble`) targeted ROS 2 Humble with Gazebo Classic. Gazebo
Classic does not exist on Jazzy, and the review for the migration also found bugs that
affected Humble. This page lists every change and why it was made.

## Layout

| v1.0.0 | v2.0.0 |
|---|---|
| one package `self_driving_bicycle` | `bicycle_description`, `bicycle_gazebo`, `bicycle_control`, `bicycle_navigation`, `bicycle_bringup` + metapackage |
| separate repository `self_driving_bicycle_hw` | `bicycle_hardware` (see [Hardware](#hardware-self_driving_bicycle_hw--bicycle_hardware)) |
| `launch/bicycle_drive.launch.py` (everything) | one launch file per concern, composed by `bicycle_bringup/launch/sim_bringup.launch.py` |
| `launch/navigation_launch.py` (copy of Humble nav2_bringup) | `bicycle_navigation/launch/navigation.launch.py` (only the servers used) |
| `urdf/properties.xacro` | `bicycle_description/config/geometry.yaml` |

## Simulation: Gazebo Classic → Gazebo Harmonic

| v1.0.0 | v2.0.0 |
|---|---|
| `gazebo_ros/gazebo.launch.py` (world never passed) | `ros_gz_sim/gz_sim.launch.py` with `world` (default `obstacles_course.sdf`), running (`-r`) |
| `spawn_entity.py` | `ros_gz_sim create` |
| `libgazebo_ros2_control.so`, `gazebo_ros2_control/GazeboSystem` | `gz_ros2_control-system`, `gz_ros2_control/GazeboSimSystem` |
| `libgazebo_ros_ray_sensor.so`, sensor `ray` | sensor `gpu_lidar` + `ros_gz_bridge` (`/scan`, `/clock`) |
| `<material>Gazebo/Black</material>` | URDF/SDF colours |
| `worlds/obstacles_course` (Classic SDF, no extension, saved `<state>`) | `worlds/obstacles_course.sdf` with Harmonic systems |
| `model://construction_cone` from the Classic database | vendored CC0 copy in `models/` |
| `gazebo_model_path` export to `share/` | environment hook adding `models/` and `worlds/` to `GZ_SIM_RESOURCE_PATH` |

## Robot description

| v1.0.0 | v2.0.0 | Why |
|---|---|---|
| `base_link` root, `base_footprint` child | `base_footprint` root → `base_link` | REP-105; `odom → base_footprint` comes from the controller |
| `base_link` at the frame centre | at the rear axle | the bicycle odometry tracks the rear axle (was off by L/2) |
| model along +Y, frame yawed −π/2 | model along +X | REP-103 |
| `steering_joint` continuous | revolute with URDF limits | Gazebo does not enforce ros2_control `min`/`max` params |
| front wheel `<mimic>`s the traction joint with `<param name="mimic">` | passive joint, state interfaces only | physically correct; the param was removed in Jazzy |
| `<resoluation>` (typo) | `<resolution>` | the values were ignored |
| scan plane 1 cm above the wheel tops | lidar on a mast at 0.65 m | avoids self-hits |
| 5 g / 1 cm steering link | 0.2 kg / 5 cm | physics stability |

## Control

| v1.0.0 | v2.0.0 | Why |
|---|---|---|
| `front_wheels_names`, `rear_wheels_names`, `front/rear_wheel_radius` | `steering_joints_names`, `traction_joints_names`, `traction_wheel_radius` | deprecated in Jazzy |
| `use_stamped_vel: false` | removed; the reference is `TwistStamped` | removed in Jazzy |
| `update_rate: 1000` | `100` | odometry and TF were published at 1 kHz |
| `base_frame_id` default (`base_link`) | `base_footprint` | `base_link` would have two parents |
| remap of `tf_odometry` on the Gazebo plugin + a C++ re-stamping node | `--controller-ros-args` remaps on the spawner to `/tf` and `/odom` | remaps on the controller_manager are deprecated; stamps are sim time now |
| `ExecuteProcess ros2 control load_controller` | `controller_manager spawner` | standard, waits for the controller_manager |
| twist_mux configured but never launched | launched, stamped, output to the controller, e-stop lock | Nav2 commands never reached the controller |

## SLAM

| v1.0.0 | v2.0.0 | Why |
|---|---|---|
| `scan_topic: /laser_controller/out` | `/scan` | SLAM received no scans |
| `map_file_name: /home/alassi69/final_ws/...` | injected from the `map` launch argument | machine-specific path |
| `mode: localization` with `async_slam_toolbox_node` | `localization_slam_toolbox_node` when `mode:=localization` | `mode` is not read; the old setup kept mapping |
| plain `Node` | upstream lifecycle-aware launch files | slam_toolbox is a lifecycle node in Jazzy |
| Humble maps | regenerated `obstacles_course.*` | frames and world changed |

## Nav2

| v1.0.0 | v2.0.0 | Why |
|---|---|---|
| NavFn, DWB `RotateToGoal`, `robot_radius: 0.22` | Smac Hybrid-A*, RPP, polygon footprint | Ackermann kinematics (see [navigation](navigation.md)) |
| plugin names `pkg/Class` | `pkg::Class` | required in Jazzy |
| `plugin_lib_names` with the default BT nodes | removed | defaults are loaded automatically; listing them registers them twice |
| default BT (Spin, BackUp) | custom BTs without them | the bicycle cannot do either |
| `Twist` velocity commands | `enable_stamped_cmd_vel: true` on every velocity node | stamped end to end |
| `use_sim_time: True` in every block | set by the launch file only | YAML values override the launch value |
| AMCL and map_server sections (unused) | removed | slam_toolbox localizes |
| — | collision_monitor | safety layer before twist_mux |

## Hardware (`self_driving_bicycle_hw` → `bicycle_hardware`)

The hardware repository was merged into this one (history kept; its last state is tagged
`hw-v1.0.0-humble`). It could not drive the bicycle:

| v1.0.0 | v2.0.0 | Why |
|---|---|---|
| own copy of the URDF (+Y layout, `base_footprint` under `base_link`, `<mimic>`) and controller YAML (Humble keys, 1 kHz) | `bicycle_description` with `hardware_plugin`/`hardware_params_file`, `bicycle_control`'s configuration | one model and one controller configuration for simulation and robot |
| hardware parameters hardcoded in the URDF (`/dev/ttyACM0`) | `config/hardware.yaml`, `device:=` | calibration without editing the model |
| `on_init(HardwareInfo)`, `export_*_interfaces()` | `on_init(HardwareComponentInterfaceParams)`, automatic interfaces | deprecated in Jazzy |
| `std::stoi` on parameters, uncaught `Open()` exception | validated configuration, errors reported | a typo or a missing device killed the controller manager |
| wheel position overwritten with `counts * 0.2` | `counts * 2 pi / 8` rad | odometry was about 3.9x too short |
| steering joint read as a wheel encoder, never commanded | stepper steps ↔ radians, both ways | the bicycle could only go straight |
| blocking reads of up to 3 s, reconnect inside the control loop, 1 kHz | one non-blocking round trip per 100 Hz cycle, `ERROR` after N failures | the control loop stalled |
| drive kept running on deactivate or error | stopped on every lifecycle exit, firmware watchdog | safety |
| firmware: `Serial.readString()` (1 s per request), heap leak and null-pointer read in `split()`, forward-only PWM 35, no steering | non-blocking parser, no heap, signed PWM, AccelStepper steering, watchdog, hard limits, versioned protocol | see [hardware_protocol.md](hardware_protocol.md) |
| libserial (undeclared) | POSIX termios | no extra dependency |
| no package dependencies, no tests | full dependencies; unit, plugin, fake-Arduino integration and consistency tests; the firmware compiled in CI | `rosdep install` works, regressions are caught |

## Repository

| v1.0.0 | v2.0.0 |
|---|---|
| `.gitignore` from ROS 1, ignoring `*.rviz` (the RViz config was never committed) | ROS 2 `.gitignore`, RViz configs committed, `.gitattributes` (LF) |
| no `exec_depend`s | full runtime dependencies per package (`rosdep install` works) |
| `src/note`, `include/.../note` placeholders | removed |
| no tests | unit, consistency, launch, lint and integration tests; CI; pre-commit |
