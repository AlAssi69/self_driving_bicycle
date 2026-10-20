# Maps

Maps of the `bicycle_gazebo` worlds, built with `sim_bringup.launch.py mode:=mapping`.
Each map is stored under the world's name, in two forms:

| File | Written by | Used by |
|---|---|---|
| `<name>.posegraph`, `<name>.data` | `slam_toolbox/serialize_map` | `mode:=localization` (`map:=<path>/<name>`, no extension) |
| `<name>.yaml`, `<name>.pgm` | `nav2_map_server map_saver_cli` | viewing, or any occupancy-grid consumer |

The map frame is the robot's start pose when the map was recorded (the world origin for
`obstacles_course`). See [docs/mapping_and_localization.md](../../docs/mapping_and_localization.md)
for how to record and save a map.
