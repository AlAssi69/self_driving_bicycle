# Navigation

Nav2 is configured in
[`bicycle_navigation/config/nav2.yaml`](../bicycle_navigation/config/nav2.yaml) and started
by [`navigation.launch.py`](../bicycle_navigation/launch/navigation.launch.py). Localization
comes from slam_toolbox (see [mapping and localization](mapping_and_localization.md)), so
Nav2 needs `slam:=true` (the default) to have a `map -> odom` transform.

## Why not the default (TurtleBot) configuration

A bicycle is an Ackermann-type vehicle: it cannot turn in place, it has a minimum turning
radius, and this one only drives forward. The default Nav2 configuration assumes a
differential-drive robot and breaks on all three points:

| Default | Problem for the bicycle | Used here |
|---|---|---|
| NavFn planner | paths with sharp corners and turns in place | **Smac Hybrid-A\***, Dubins motion model |
| DWB with `RotateToGoal` | rotates in place to align | **Regulated Pure Pursuit**, `use_rotate_to_heading: false` |
| `robot_radius: 0.22` | the bicycle is 1.58 m long | polygon `footprint` |
| Spin / BackUp recoveries | impossible manoeuvres; a missing action server fails the goal | behavior trees with clearing + wait recoveries |

## Minimum turning radius

With wheelbase L = 0.98 m and steering limit δ = 0.7854 rad (45°), the rear axle turns on
a circle of radius

    R_min = L / tan(δ) = 0.98 m

The planner uses `minimum_turning_radius: 1.1` m, which leaves margin because the steering
needs time to reach its angle and the wide wheels scrub (they turn slightly wider than
the kinematic model). Related limits:

- RPP `min_lookahead_dist: 1.0` m (≥ R_min), so the commanded curvature stays feasible.
- Velocity smoother: max yaw rate 0.36 rad/s = 0.4 m/s / 1.1 m. A rotation command without
  forward speed would make the controller request full steering lock.
- `analytic_expansion_max_length: 6.0` m (≥ 5 R) so Smac connects to the goal with Dubins
  curves.

## Footprint and inflation

The robot frame `base_footprint` is under the rear axle, so the footprint is asymmetric:

    x from -0.30 to 1.28 m (rear wheel edge to front wheel edge, plus 5 cm)
    y from -0.15 to 0.15 m (wheel width 0.20 m, plus 5 cm)

Its circumscribed radius (the farthest footprint corner from the rear axle) is 1.29 m. Smac
Hybrid-A* only uses its fast collision check when the inflation radius covers it, so both
costmaps use `inflation_radius: 1.4`. With a smaller radius the planner falls back to full
footprint checks for every expansion and times out on longer paths.

Goals must leave room for the whole footprint: a goal facing a wall must be at least 1.3 m
from it, and a goal near the edge of a partly mapped area can be unreachable because part
of the footprint would be off the map.

## Speed limits

| Limit | Value | Where |
|---|---|---|
| Rear wheel (hardware) | 2.0 rad/s × 0.25 m = 0.5 m/s | `geometry.yaml` (URDF joint limit) |
| Nav2 maximum | 0.4 m/s | velocity smoother, RPP `desired_linear_vel` |
| Reverse | none | velocity smoother `min_velocity[0] = 0`, RPP `allow_reversing: false` |

## Safety

- **collision_monitor** sits after the velocity smoother. It projects the footprint along
  the commanded velocity and slows the bicycle so it never reaches an obstacle seen in
  `/scan` within 1.2 s.
- **twist_mux** gives teleop priority over Nav2, and the `e_stop` lock priority over both.

## Behavior trees

[`behavior_trees/`](../bicycle_navigation/behavior_trees) contains the Jazzy
`navigate_to_pose` and `navigate_through_poses` replanning-and-recovery trees with Spin and
BackUp removed. Recovery is: clear both costmaps, then wait 5 s, in a round robin. Only the
`wait` behavior is loaded in the behavior server.

## Tuning

| Symptom | Parameter |
|---|---|
| Planner too slow or "exceeded maximum iterations" | goal too close to obstacles or the map edge; `max_planning_time`, `angle_quantization_bins` |
| Path cuts corners the bicycle cannot follow | increase `minimum_turning_radius` |
| Oscillation around the path | increase `lookahead_dist` / `min_lookahead_dist` |
| Too slow near obstacles | `time_before_collision` (collision monitor), `inflation_radius`/`cost_scaling_factor` |
| Stops short of the goal | `xy_goal_tolerance`, `yaw_goal_tolerance` (general_goal_checker) |

### Alternative controller

The MPPI controller (`nav2_mppi_controller::MPPIController`) has an Ackermann motion model
that enforces the minimum turning radius itself (`AckermannConstraints.min_turning_r`). It
is heavier to tune and compute; RPP follows the curvature-limited Smac paths well enough for
this robot.
