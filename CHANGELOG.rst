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
* Real robot: the ``self_driving_bicycle_hw`` repository merged in as ``bicycle_hardware``
  (history kept, tag ``hw-v1.0.0-humble``), rewritten for Jazzy:

  * ros2_control plugin on the Jazzy API, validated parameters, no exceptions out of the
    lifecycle, correct wheel odometry (was about 3.9x too short), steering actually
    commanded, one non-blocking round trip per cycle instead of blocking reads of up to
    3 s, drive stopped on every lifecycle exit; POSIX serial port (no libserial).
  * New Arduino firmware: non-blocking requests (each one used to take 1 s), no heap
    (the old parser leaked memory on every message and dereferenced a null pointer),
    signed PWM in both directions, the steering stepper, a 300 ms watchdog, hard limits,
    and a versioned protocol shared with the plugin.
  * Uses ``bicycle_description`` (new ``hardware_params_file`` argument) and
    ``bicycle_control`` instead of its own outdated URDF and controller configuration.
  * ``robot_bringup.launch.py`` (same controllers, SLAM and Nav2 as the simulation),
    ``rviz.launch.py``, ``teleop_keyboard --real``, the YDLIDAR X4 Pro driver
    (``robot.repos``), udev rules; the simulated lidar now models the X4 Pro.
  * ``fake_mcu``: the firmware's logic on a pseudo-terminal, for tests and dry runs.

* Tests (URDF, cross-package consistency, launch files, ament linters, C++ unit tests, the
  real-robot stack against the fake Arduino, headless Gazebo integration), GitHub Actions
  CI (including the firmware build), pre-commit, and documentation.

1.0.0 (2024-08-22)
------------------
Humble / Gazebo Classic version (tag ``v1.0.0-humble``): URDF model, bicycle steering
controller, slam_toolbox and Nav2 configuration.
