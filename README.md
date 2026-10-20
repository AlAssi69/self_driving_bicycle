# Self-Driving Bicycle

[![CI](https://github.com/AlAssi69/self_driving_bicycle/actions/workflows/ci.yml/badge.svg)](https://github.com/AlAssi69/self_driving_bicycle/actions/workflows/ci.yml)

A self-driving bicycle on **ROS 2 Jazzy**, in simulation (**Gazebo Harmonic**) and on the
real robot (Arduino Uno, DC drive motor, steering stepper, YDLIDAR X4 Pro): robot
description, ros2_control, SLAM (slam_toolbox) and autonomous navigation (Nav2), tuned for
a bicycle's forward-only, minimum-turning-radius kinematics.

It is part of our graduation project at Damascus University, Department of Computer and
Automation Engineering.

## Features

- URDF/xacro model with every dimension in one file
  ([`geometry.yaml`](bicycle_description/config/geometry.yaml)); the ros2_control hardware
  plugin is an argument, so the same description serves Gazebo and the real robot.
- Gazebo Harmonic world with a 2D lidar, rear-wheel drive and front-wheel steering through
  `gz_ros2_control` and the `bicycle_steering_controller`.
- The real bicycle runs the same controllers, SLAM and Nav2: a ros2_control plugin talks to
  an Arduino firmware over a versioned serial protocol, with a watchdog and hard limits on
  the Arduino ([docs/hardware.md](docs/hardware.md)).
- Velocity commands multiplexed by `twist_mux` (Nav2 < teleop < latched e-stop).
- slam_toolbox mapping and localization.
- Nav2 configured for Ackermann-type motion: Smac Hybrid-A* planner, Regulated Pure Pursuit
  controller, polygon footprint, collision monitor, behavior trees without spin/back-up.
- Tests: URDF checks, cross-package consistency checks, launch smoke tests, ROS linters,
  C++ unit tests, the real-robot stack against a simulated Arduino, and a headless Gazebo
  integration test, all in CI, which also compiles the firmware.

## Requirements

| | |
|---|---|
| OS | Ubuntu 24.04 (native or WSL2) |
| ROS | ROS 2 Jazzy (`ros-jazzy-desktop`) |
| Simulator | Gazebo Harmonic (`ros-jazzy-ros-gz`) |

Everything else is installed by `rosdep` from the packages' manifests.

## Install

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/AlAssi69/self_driving_bicycle.git
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Re-source `install/setup.bash` after each build in every terminal you use: it also tells
Gazebo where the worlds and models are.

## Quick start

| Goal | Command |
|---|---|
| Inspect the model (no simulator) | `ros2 launch bicycle_description display.launch.py` |
| Drive the bicycle in Gazebo | `ros2 launch bicycle_bringup sim_bringup.launch.py slam:=false` |
| ... with the keyboard (2nd terminal) | `ros2 run bicycle_bringup teleop_keyboard` |
| Build a map | `ros2 launch bicycle_bringup sim_bringup.launch.py` (mapping is the default), then [save it](docs/mapping_and_localization.md#save-the-map) |
| Navigate while mapping | `ros2 launch bicycle_bringup sim_bringup.launch.py nav:=true` |
| Navigate in the saved map | `ros2 launch bicycle_bringup sim_bringup.launch.py mode:=localization nav:=true` |
| Emergency stop / release | `ros2 topic pub -r 5 /e_stop std_msgs/msg/Bool "{data: true}"` / `... "{data: false}"` |

With Nav2 running, use RViz's **2D Goal Pose** tool to send a goal. The bicycle cannot
turn in place, so give it room: its footprint extends 1.28 m ahead of the rear axle and it
turns with a radius of at least 1.1 m.

### On the real bicycle

After [installing, flashing and the first-run checks](docs/hardware.md):

| Goal | Command |
|---|---|
| Drive with the keyboard | `ros2 launch bicycle_bringup robot_bringup.launch.py slam:=false`, then `ros2 run bicycle_bringup teleop_keyboard --real` |
| Map / navigate | `ros2 launch bicycle_bringup robot_bringup.launch.py` (`mode:=`, `map:=`, `nav:=true` as in simulation) |
| Watch from a development machine | `ros2 launch bicycle_bringup rviz.launch.py config:=navigation` |
| Dry run without the bike | `ros2 run bicycle_hardware fake_mcu --link /tmp/bicycle_fake_mcu`, then `robot_bringup.launch.py device:=/tmp/bicycle_fake_mcu lidar:=false` |

### Main launch arguments (`sim_bringup.launch.py`)

| Argument | Default | Description |
|---|---|---|
| `slam` | `true` | Run slam_toolbox (Nav2 needs its `map -> odom`). |
| `mode` | `mapping` | `mapping` builds a new map, `localization` localizes in `map`. |
| `map` | `bicycle_navigation/maps/obstacles_course` | Serialized slam_toolbox map, without extension. |
| `nav` | `false` | Run the Nav2 servers. |
| `rviz` | `true` | Start RViz. |
| `world` | `obstacles_course.sdf` | World file name (searched in `GZ_SIM_RESOURCE_PATH`) or path. |
| `headless` | `false` | Gazebo server only, no GUI. |
| `x`, `y`, `yaw` | `0.0` | Spawn pose. |

`ros2 launch bicycle_bringup sim_bringup.launch.py --show-args` lists every argument,
including those of the included launch files.

## Packages

| Package | Contents |
|---|---|
| [`bicycle_description`](bicycle_description) | URDF/xacro, `geometry.yaml`, robot_state_publisher launch, model viewer |
| [`bicycle_gazebo`](bicycle_gazebo) | Gazebo worlds and models, ROS-Gazebo bridge, simulation launch |
| [`bicycle_hardware`](bicycle_hardware) | Real robot: ros2_control plugin, Arduino firmware, lidar configuration, udev rules |
| [`bicycle_control`](bicycle_control) | Controllers, `twist_mux` and e-stop configuration, controller spawning |
| [`bicycle_navigation`](bicycle_navigation) | slam_toolbox and Nav2 configuration, behavior trees, maps |
| [`bicycle_bringup`](bicycle_bringup) | Top-level launch, RViz configurations, teleop, cross-package tests |
| `self_driving_bicycle` | Metapackage depending on all of the above |

## Documentation

- [Architecture](docs/architecture.md): packages, launch composition, TF tree, topics, velocity pipeline
- [Configuration](docs/configuration.md): the single source of truth and how to change the robot
- [Mapping and localization](docs/mapping_and_localization.md)
- [Navigation](docs/navigation.md): planner and controller choices for a bicycle, tuning
- [Real bicycle](docs/hardware.md): wiring, install, flashing, first run, calibration, and
  the [serial protocol](docs/hardware_protocol.md)
- [Migration from Humble](docs/migration_humble_to_jazzy.md): what changed and why
- [Troubleshooting](docs/troubleshooting.md): WSL, rendering, Gazebo, TF and Nav2 issues
- [Contributing](CONTRIBUTING.md) and [changelog](CHANGELOG.rst)

## Testing

```bash
colcon test --ctest-args -LE sim_integration   # unit, consistency, launch, lint, fake-Arduino tests
colcon test --packages-select bicycle_bringup --ctest-args -L sim_integration  # headless Gazebo
colcon test-result --verbose
pre-commit run --all-files                      # formatting, YAML/XML, spelling
```

## Authors

[Ashraf AlAssi](https://github.com/AlAssi69/),
[Mhd. Osama Keakaty](https://github.com/Osama-keakaty) and
[Sarah AlAhmad](https://github.com/SarahAlahmad).

## License

[Apache License 2.0](LICENSE). The vendored Construction Cone model is CC0 (Gazebo Fuel).
