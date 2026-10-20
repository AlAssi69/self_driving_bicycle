cmake_minimum_required(VERSION 3.8)
project(bicycle_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()
