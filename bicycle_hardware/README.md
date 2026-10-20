# bicycle_hardware

Hardware layer of the real bicycle: the robot's counterpart of `bicycle_gazebo`.

- `bicycle_hardware/BicycleSystemHardware`: ros2_control system plugin for the Arduino Uno
  over USB serial. Steering joint: position command (stepper). Traction joint: velocity
  command (DC motor, IR wheel encoder). Front wheel: state only, estimated from the rear
  wheel and the steering angle.
- `firmware/bicycle_uno/`: the Arduino sketch (AccelStepper). Wiring and hard limits in
  `config.hpp`; the serial protocol in `protocol.hpp`, shared with the plugin.
- `config/hardware.yaml`: serial link and calibration, passed to the description as
  `hardware_params_file`. `config/ydlidar.yaml`: YDLIDAR X4 Pro driver parameters.
- `launch/hardware.launch.py`: `ros2_control_node` (with `bicycle_control`'s controller
  configuration) and, with `lidar:=true`, the lidar driver from `robot.repos`.
- `udev/99-bicycle.rules`: `/dev/bicycle_arduino` and `/dev/ydlidar` for the `dialout` group.
- `fake_mcu`: the firmware's logic on a pseudo-terminal, for tests and dry runs
  (`ros2 run bicycle_hardware fake_mcu --link /tmp/bicycle_fake_mcu`).

Start the whole robot with `ros2 launch bicycle_bringup robot_bringup.launch.py`. Wiring,
installation, flashing, first run and calibration: [docs/hardware.md](../docs/hardware.md).

This package was the separate repository `self_driving_bicycle_hw` (history kept; its last
state is tagged `hw-v1.0.0-humble`).
