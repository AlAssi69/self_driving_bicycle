cmake_minimum_required(VERSION 3.8)
project(bicycle_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_consistency test/test_consistency.py)
  ament_add_pytest_test(test_launch_files test/test_launch_files.py)

  # Starts Gazebo headless; slow, so it carries its own label (ctest -L / -LE sim_integration).
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/integration/test_sim_integration.py
    TARGET test_sim_integration
    TIMEOUT 300
  )
  set_tests_properties(test_sim_integration PROPERTIES LABELS sim_integration)
endif()

ament_package()
