# Contributing

## Workflow

1. Create a branch from `main` (`feature/...`, `fix/...`).
2. Make small, focused commits with imperative subjects (`feat(navigation): ...`,
   `fix(control): ...`, `docs: ...`).
3. Make sure tests, linters and pre-commit pass locally, then open a pull request. CI runs
   the same checks.

## Set up

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
pip install pre-commit   # or: sudo apt install pre-commit
cd src/self_driving_bicycle && pre-commit install
```

## Checks

```bash
colcon test --ctest-args -LE sim_integration    # unit, consistency, launch, ament linters
colcon test --packages-select bicycle_bringup --ctest-args -L sim_integration  # Gazebo, ~1 min
colcon test-result --verbose
pre-commit run --all-files
```

- Python follows `ament_flake8` (99 columns, single quotes, Google import order) and
  `ament_pep257`; `ruff format` keeps the formatting consistent.
- Every Python file starts with the Apache-2.0 header (checked by `ament_copyright`).
- XML (URDF/xacro/SDF) is checked by `ament_xmllint` and pre-commit.

## Where things go

| Change | Place |
|---|---|
| Robot dimensions, masses, limits, sensor mount | `bicycle_description/config/geometry.yaml`, then the repeated values listed in [docs/configuration.md](docs/configuration.md) |
| Links, joints, sensors | `bicycle_description/urdf/` |
| Worlds and models | `bicycle_gazebo/worlds/`, `bicycle_gazebo/models/` (vendor models; no online URIs) |
| Topics bridged from Gazebo | `bicycle_gazebo/config/gz_bridge.yaml` |
| Controllers, command sources, e-stop | `bicycle_control/config/` |
| SLAM, Nav2, behavior trees, maps | `bicycle_navigation/` |
| New launch file | the package that owns the concern; add it to `bicycle_bringup/test/test_launch_files.py` |

Keep the dependency direction (description ← gazebo/control ← navigation ← bringup), declare
every runtime dependency in `package.xml`, and do not hardcode `use_sim_time` or absolute
paths in configuration files.
