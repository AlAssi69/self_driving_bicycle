^^^^^^^^^
Changelog
^^^^^^^^^

2.0.0 (unreleased)
------------------
Port to ROS 2 Jazzy and Gazebo Harmonic, split into packages, and fix the bugs found on the
way. Details: ``docs/migration_humble_to_jazzy.md``.

* Split the single package into ``bicycle_description``, ``bicycle_gazebo``,
  ``bicycle_control``, ``bicycle_navigation``, ``bicycle_bringup`` and the
  ``self_driving_bicycle`` metapackage, with full runtime dependencies.
* Gazebo Classic replaced by Gazebo Harmonic (``ros_gz_sim``, ``ros_gz_bridge``,
  ``gz_ros2_control``, ``gpu_lidar``); the obstacle course world is actually loaded now.
* Robot description: REP-103/105 frames with ``base_footprint`` under the rear axle as the
  root, model along +X, revolute steering with URDF limits, passive front wheel, fixed lidar
  tag typo, lidar mast, all dimensions in ``geometry.yaml``, injectable hardware plugin.
* Control: Jazzy controller parameters, stamped commands, 100 Hz, ``/odom`` and ``/tf``
  remaps through the spawner, twist_mux actually launched (Nav2 commands never reached the
  controller), latched e-stop.
* SLAM: correct scan topic (SLAM received no scans), no absolute map path, real localization
  mode (``mode: localization`` was ignored and the robot kept mapping), regenerated map.
* Nav2: configuration for a forward-only Ackermann-type robot (Smac Hybrid-A*, Regulated
  Pure Pursuit, polygon footprint, collision monitor, behavior trees without spin/back-up),
  Jazzy plugin names, stamped velocity chain, small project-owned launch file.
* Committed RViz configurations (``*.rviz`` used to be gitignored), ROS 2 ``.gitignore``.
* Tests (URDF, cross-package consistency, launch files, ament linters, headless Gazebo
  integration), GitHub Actions CI, pre-commit, and documentation.

1.0.0 (2024-08-22)
------------------
Humble / Gazebo Classic version (tag ``v1.0.0-humble``): URDF model, bicycle steering
controller, slam_toolbox and Nav2 configuration.
