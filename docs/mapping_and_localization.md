# Mapping and localization

slam_toolbox provides the `map` frame and the `/map` occupancy grid in both modes. It is
configured in [`bicycle_navigation/config/slam.yaml`](../bicycle_navigation/config/slam.yaml)
and started by [`slam.launch.py`](../bicycle_navigation/launch/slam.launch.py):

| `mode` | Node | Behaviour |
|---|---|---|
| `mapping` (default) | `async_slam_toolbox_node` | builds a new map from the robot's start pose |
| `localization` | `localization_slam_toolbox_node` | loads the serialized map given by `map` and localizes in it |

## Build a map

1. Start the simulation in mapping mode:

   ```bash
   ros2 launch bicycle_bringup sim_bringup.launch.py
   ```

2. Drive the bicycle around the whole area. Use either:
   - the keyboard: `ros2 run bicycle_bringup teleop_keyboard` in a second terminal, or
   - Nav2 (start with `nav:=true`) and RViz's **2D Goal Pose** tool, sending goals towards
     unexplored space. Nav2 plans through unknown space while mapping.

   Cover the area from different directions; slam_toolbox only marks cells it has seen in
   more than one scan, so the map stays sparse while the robot stands still.

### Save the map

Save both forms, using the same path without extension:

```bash
MAP=~/ros2_ws/src/self_driving_bicycle/bicycle_navigation/maps/my_map
# Pose graph for slam_toolbox localization (<name>.posegraph + <name>.data)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP'}"
# Occupancy grid for viewing or other tools (<name>.yaml + <name>.pgm)
ros2 run nav2_map_server map_saver_cli -f $MAP --ros-args -p use_sim_time:=true
```

Rebuild (`colcon build`) to install new map files into the package share, or pass the
absolute path as `map:=` (no rebuild needed).

The shipped `obstacles_course` map was made this way: in mapping mode, Nav2 drove six
waypoints around the cone and along both walls, with a U-turn at each end of the arena.

## Localize in a saved map

```bash
ros2 launch bicycle_bringup sim_bringup.launch.py mode:=localization nav:=true
# another map:
ros2 launch bicycle_bringup sim_bringup.launch.py mode:=localization nav:=true map:=/abs/path/my_map
```

The localization node assumes the robot starts at `map_start_pose` (`[0, 0, 0]`), the pose
where mapping started, which is also the default spawn pose. If the robot starts elsewhere
(for example `x:=2.0 y:=1.0`), set its pose with RViz's **2D Pose Estimate** tool, or change
`map_start_pose` in `slam.yaml`.

## Notes

- The `map` frame is the robot's start pose when the map was recorded. Keep spawn poses
  consistent between mapping and localization.
- Maps depend on the robot's frames and lidar mount. After changing `geometry.yaml` (lidar
  position, wheelbase), record the maps again.
- In localization mode slam_toolbox keeps a short rolling buffer of new scans
  (`scan_buffer_size`) to match against the map; it does not extend the saved map.
