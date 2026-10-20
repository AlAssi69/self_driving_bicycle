# bicycle_control

ros2_control controllers and velocity command multiplexing, independent of the simulator.

- `config/bicycle_controller.yaml`: controller_manager (100 Hz), `joint_state_broadcaster`,
  `bicycle_controller` (`bicycle_steering_controller`: rear-wheel drive, front steering,
  odometry of the rear axle in `odom -> base_footprint`).
- `config/twist_mux_topics.yaml`, `config/twist_mux_locks.yaml`: command sources and the
  e-stop lock.
- `launch/control.launch.py`: spawns the controllers in `/controller_manager` and starts
  twist_mux. Arguments: `use_sim_time` (`false`), `controller_manager_timeout` (`120`).

| Topic | Type | Role |
|---|---|---|
| `/cmd_vel` | `TwistStamped` | Nav2 commands, priority 10 |
| `/cmd_vel_teleop` | `TwistStamped` | teleop commands, priority 100 |
| `/e_stop` | `Bool` | latched lock, priority 255: `true` stops, `false` releases |
| `/bicycle_controller/reference` | `TwistStamped` | twist_mux output, the controller's only input |
| `/odom`, `/tf` | `Odometry`, `TFMessage` | controller odometry |
