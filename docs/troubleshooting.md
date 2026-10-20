# Troubleshooting

## Gazebo cannot find the world or models

`Unable to find uri[model://box]` or `obstacles_course.sdf` not found: the environment hook
of `bicycle_gazebo` was not sourced. Run `source install/setup.bash` after building, in every
terminal. Check with `echo $GZ_SIM_RESOURCE_PATH`.

## Two simulations interfere

gz-transport ignores `ROS_DOMAIN_ID`. If two Gazebo instances run on the same machine (for
example a leftover `gz sim` process and a new launch), their `/clock` and `/scan` mix and
TF, the controllers and Nav2 misbehave (Nav2's `controller_server` fails to activate). Kill
leftovers (`pkill -f "gz sim"`) or give each instance its own partition:

```bash
export GZ_PARTITION=my_sim   # in every terminal of that simulation
```

The integration test uses its own partition for this reason.

## WSL2

- **`ros2` CLI commands hang** (`ros2 control list_controllers`, `ros2 topic echo`, ...):
  the ROS 2 daemon can get stuck under WSL. Run `ros2 daemon stop`, or add `--no-daemon`
  where the command supports it. A direct call always works:
  `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers`.
- **Gazebo GUI is black or crashes**: try `export LIBGL_ALWAYS_SOFTWARE=1`, or run headless
  (`headless:=true`) and watch in RViz. The lidar still works headless
  (`--headless-rendering`).
- **Gazebo topics not visible between terminals**: set `export GZ_IP=127.0.0.1`.
- **ROS topics do not arrive between processes** (the controller manager keeps waiting for
  `robot_description`, spawners time out on `list_controllers`, a `listener` hears no
  `talker`), seen on a WSL machine with a VPN-style `/32` interface and Docker bridges: Fast
  DDS's default transport mix drops data between processes there, while either transport
  alone works. Restrict it in every terminal:
  `export FASTDDS_BUILTIN_TRANSPORTS=SHM` (single machine) or `=UDPv4` (robot and a
  development machine). Check with `ros2 run demo_nodes_cpp talker` and `listener`.

## Controllers do not start

- The spawners wait up to `controller_manager_timeout` (120 s) for gz_ros2_control. If they
  time out, check that `/robot_description` is published (robot_state_publisher must run,
  with `use_sim:=true`) and that the `gz_ros2_control` package is installed.
- `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers`
  should list `joint_state_broadcaster` and `bicycle_controller` as `active`.

## The bicycle does not move

- Commands must be `geometry_msgs/TwistStamped` on `/cmd_vel_teleop` or `/cmd_vel`; the
  controller only accepts what twist_mux forwards.
- The command stamp must be on the simulation clock. A wall-clock stamp is far in the future
  for the controller and never times out, and a zero stamp is replaced by "now" with a
  warning. Use `ros2 run bicycle_bringup teleop_keyboard`, which sets `use_sim_time`.
- Is the e-stop engaged? Publish `false`:
  `ros2 topic pub -r 5 /e_stop std_msgs/msg/Bool "{data: false}"` (stop it after a second).
- `ros2 topic pub --once` can exit before the message is delivered; for the e-stop use a
  continuous publisher (`-r 5`) as above.

## Real robot

- **`Cannot open the Arduino serial port`**: check `ls -l /dev/bicycle_arduino` (udev rule
  installed?) and that your user is in `dialout` (`groups`); in WSL attach the device with
  usbipd and pass `device:=/dev/ttyACM0` ([hardware.md](hardware.md#developing-in-wsl)).
- **`No answer from the Arduino ... is firmware/bicycle_uno flashed?`**: the firmware is
  missing, another program (a serial monitor) holds the port, or the baud rate differs from
  `protocol.hpp`. Try `V` in a serial monitor at 115200 baud.
- **`Firmware '...' version ... expected`**: the Arduino runs an older firmware; flash the
  one from this checkout.
- **`The firmware watchdog stopped the drive`**: commands arrive more than 300 ms apart; the
  computer is overloaded or the USB link drops data.
- **`Serial link to the Arduino lost`**: the controllers stop and the hardware goes to
  `unconfigured`. Reconnect, then restart the launch (or reconfigure the component with
  `ros2 control set_hardware_component_state BicycleSystem active`).
- **`package 'ydlidar_ros2_driver' not found`**: import and build `robot.repos`
  ([hardware.md](hardware.md#install-on-the-robots-computer)), or start without the lidar
  (`lidar:=false`).

## SLAM

- **The map stays almost empty while standing still**: slam_toolbox only marks a cell once
  it has been seen from more than one scan. Drive a little.
- **Localization does not start**: the `map` argument must point to an existing
  `<name>.posegraph`/`<name>.data` pair, without extension. The robot must start at
  `map_start_pose` (the pose where the map was started, the origin by default), or be
  placed with RViz's **2D Pose Estimate**.

## Nav2

- **Goal fails with "exceeded maximum iterations" / timeout**: the goal footprint (1.58 m,
  mostly ahead of the rear axle) collides with an obstacle, a wall or the edge of the known
  map, or the goal needs a tighter turn than 1.1 m. Move the goal away from obstacles and
  give it a heading the bicycle can reach. See [navigation](navigation.md#footprint-and-inflation).
- **"Inflation radius ... smaller than the circumscribed radius"**: the global costmap's
  `inflation_radius` must be at least the footprint's circumscribed radius (1.29 m).
- **Nav2 servers fail to activate**: they wait up to `initial_transform_timeout` (60 s) for
  `map -> odom`. Start Nav2 with SLAM (`slam:=true`), and make sure only one simulation runs.
