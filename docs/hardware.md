# The real bicycle

`bicycle_hardware` connects the same controllers, SLAM and Nav2 as the simulation to the
real bicycle. This page covers wiring, installation, flashing, the first run and
calibration. The serial protocol is in [hardware_protocol.md](hardware_protocol.md).

> Status: the firmware compiles for the Uno and the whole robot stack is tested against a
> simulated Arduino that runs the firmware's own logic (`fake_mcu`). It has not run on the
> bike yet: the [first-run checklist](#first-run-wheels-off-the-ground) is its acceptance
> test.

## Components

| Part | Model | Connection |
|---|---|---|
| Computer | Ubuntu 24.04, ROS 2 Jazzy | USB to the Arduino and the lidar |
| Microcontroller | Arduino Uno, [`firmware/bicycle_uno`](../bicycle_hardware/firmware/bicycle_uno) | USB serial, 115200 baud |
| Drive | DC motor on a BTS7960 (IBT-2) H-bridge | Arduino PWM |
| Wheel encoder | IR slot sensor on the rear wheel, 8 slots | Arduino interrupt pin |
| Steering | Stepper motor on a STEP/DIR/ENABLE driver | Arduino digital pins |
| Lidar | YDLIDAR X4 Pro | USB (CP210x), 128000 baud |

## Wiring (Arduino Uno)

All pins are set in [`config.hpp`](../bicycle_hardware/firmware/bicycle_uno/config.hpp).

| Uno pin | Connects to | `config.hpp` |
|---|---|---|
| D2 | IR sensor output | `kEncoderPin` |
| D6 (PWM) | BTS7960 LPWM (forward) | `kDriveForwardPwmPin` |
| D3 (PWM) | BTS7960 RPWM (reverse) | `kDriveReversePwmPin` |
| D5 | BTS7960 L_EN | `kDriveForwardEnablePin` |
| D8 | BTS7960 R_EN | `kDriveReverseEnablePin` |
| D9 | stepper driver STEP (PUL) | `kSteeringStepPin` |
| D10 | stepper driver DIR | `kSteeringDirPin` |
| D11 | stepper driver ENABLE (ENA) | `kSteeringEnablePin` |
| GND | common ground of both drivers and the sensor | |

The drive and encoder pins are the ones the original firmware used. The original firmware
had no stepper code, so **the three stepper pins are placeholders: set them to the bike's
wiring before flashing**, together with `kSteeringEnableActiveLow` (most drivers enable on
LOW) and `kSteeringInvertDirection`.

## Install on the robot's computer

The lidar driver is not released for Jazzy; [`robot.repos`](../robot.repos) pins its source.

```bash
sudo apt install python3-vcstool
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/AlAssi69/self_driving_bicycle.git
vcs import < self_driving_bicycle/robot.repos
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --packages-select ydlidar_sdk
# The SDK's CMake config leaves its library directory empty: point the linker at it.
LIBRARY_PATH=$PWD/install/ydlidar_sdk/lib colcon build --symlink-install
source install/setup.bash
```

### Device access without root

[`udev/99-bicycle.rules`](../bicycle_hardware/udev/99-bicycle.rules) gives the Arduino
and the lidar stable names (`/dev/bicycle_arduino`, `/dev/ydlidar`) that only the
`dialout` group can open:

```bash
sudo cp src/self_driving_bicycle/bicycle_hardware/udev/99-bicycle.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER"   # then log out and in
ls -l /dev/bicycle_arduino /dev/ydlidar
```

## Flash the firmware

With [arduino-cli](https://arduino.github.io/arduino-cli/) (the versions CI uses):

```bash
arduino-cli core install arduino:avr@1.8.6
arduino-cli lib install AccelStepper@1.64
SKETCH=$(ros2 pkg prefix bicycle_hardware)/share/bicycle_hardware/firmware/bicycle_uno
arduino-cli compile --fqbn arduino:avr:uno "$SKETCH"
arduino-cli upload --fqbn arduino:avr:uno -p /dev/bicycle_arduino "$SKETCH"
```

Or open `bicycle_uno.ino` in the Arduino IDE, install AccelStepper from the library manager
and upload. To check it, open a serial monitor at 115200 baud with newline line endings
(`arduino-cli monitor -p /dev/bicycle_arduino -c baudrate=115200`):

```text
V          ->  V bicycle_uno 1
Q 1        ->  S 1 0 0 0 5        (5: watchdog stopped the drive, stepper enabled)
```

## Dry run without the bike

`fake_mcu` emulates the Arduino on a pseudo-terminal, running the firmware's own request
handling against a simulated motor, encoder and stepper:

```bash
ros2 run bicycle_hardware fake_mcu --link /tmp/bicycle_fake_mcu
ros2 launch bicycle_bringup robot_bringup.launch.py device:=/tmp/bicycle_fake_mcu lidar:=false slam:=false
ros2 run bicycle_bringup teleop_keyboard --real
```

`fake_mcu` prints the PWM and steering steps it receives; `/odom` and `/joint_states`
follow the simulated encoder and stepper.

## First run (wheels off the ground)

1. Lift the bike so the rear wheel turns freely. Centre the handlebar by hand: the
   steering's zero is its position when the Arduino starts.
2. Start the robot without SLAM:
   `ros2 launch bicycle_bringup robot_bringup.launch.py slam:=false`.
   Expect `Connected to bicycle_uno on /dev/bicycle_arduino` and both controllers active
   (`ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers`).
3. Keep an e-stop ready in another terminal:
   `ros2 topic pub -r 5 /e_stop std_msgs/msg/Bool "{data: true}"` (Ctrl-C, then publish
   `false`, to release).
4. `ros2 run bicycle_bringup teleop_keyboard --real`, then `i` (forward) at low speed. The
   wheel must turn forward; otherwise swap the motor leads (or the two drive PWM pins).
5. `u` (forward and left) must turn the handlebar to the left (counter-clockwise seen from
   above); otherwise flip `kSteeringInvertDirection`.
6. Unplug the Arduino's USB cable while driving: the motor stops within 0.3 s (firmware
   watchdog), the plugin reports the lost link and the controllers stop.
7. Check the encoder: one wheel revolution moves `traction_joint` in `/joint_states` by
   2π (6.28) rad.
8. Check the lidar from a development machine (`ros2 launch bicycle_bringup rviz.launch.py`):
   an object straight ahead of the bike must appear ahead of `laser_frame` (+x); otherwise
   flip `reversion` or `inverted` in `config/ydlidar.yaml`.

Then drive on the ground with teleop, and map as in the simulation
([mapping_and_localization.md](mapping_and_localization.md)) with
`robot_bringup.launch.py` instead of `sim_bringup.launch.py`.

## Calibration

| Value | File | How to set it |
|---|---|---|
| `encoder_counts_per_rev` | `config/hardware.yaml` | Slots on the encoder disk; check with one wheel revolution (step 7). |
| `steering_steps_per_rev`, `steering_microsteps`, `steering_gear_ratio` | `config/hardware.yaml` | Stepper (usually 200 steps/rev), the driver's microstep switches, and the gearing to the fork. Check: a 0.5 rad command turns the handlebar by 0.5 rad (29°). |
| `kHardLimits` | `firmware/bicycle_uno/config.hpp` | Safety caps in steps; keep them just above the URDF steering limit (a test checks it). |
| `steering_max_acceleration` | `config/hardware.yaml` | Lower it if the stepper skips steps when the steering reverses. |
| `drive_deadband` | `config/hardware.yaml` | Smallest PWM that turns the lifted wheel. |
| `drive_feedforward` | `config/hardware.yaml` | Drive at a constant PWM, read the wheel speed (rad/s) in `/joint_states`: (PWM − deadband) / speed. |
| `drive_kp`, `drive_ki` | `config/hardware.yaml` | Leave at 0 unless the speed sags under load; keep them small (the encoder is coarse). |
| `drive_max_pwm` | `config/hardware.yaml` | Speed cap for safety; must allow the 0.5 m/s limit (a test checks it). |
| wheelbase, wheel radius, lidar height | `bicycle_description/config/geometry.yaml` | Measure the bike; the simulation follows. |

`ros2 launch bicycle_bringup robot_bringup.launch.py hardware_params_file:=<copy>.yaml`
uses a calibrated copy without editing the installed file.

## Developing in WSL

WSL does not see USB devices by default. With [usbipd-win](https://github.com/dorssel/usbipd-win):

```powershell
winget install usbipd
usbipd list                          # find the Arduino's / lidar's BUSID
usbipd bind --busid <BUSID>          # once, in an administrator terminal
usbipd attach --wsl --busid <BUSID>  # after each plug-in
```

The devices appear as `/dev/ttyACM0` (Arduino) and `/dev/ttyUSB0` (lidar); WSL usually does
not run the udev rules, so pass `device:=/dev/ttyACM0` and set `port: /dev/ttyUSB0` in a
copy of `ydlidar.yaml` (`lidar_params_file:=`).

## Known limitations

- **Wheel encoder.** The single-channel IR sensor cannot sense direction (each pulse takes
  the sign of the drive command), and 8 slots mean 0.196 m of travel per count. The speed
  comes from the time between pulses and decays to zero after `encoder_timeout_s`; the
  odometry is accurate to about one count per start and stop. SLAM corrects it. A
  quadrature encoder with more counts would remove both limits.
- **Steering feedback.** The steering angle is the step count AccelStepper issued, not a
  measurement: if the stepper stalls or skips steps, the angle drifts until the steering is
  re-centred (restart the Arduino, or send `Z`).
- **No homing switch.** Centre the handlebar before switching on.
