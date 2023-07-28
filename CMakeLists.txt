cmake_minimum_required(VERSION 3.8)
project(cyecca_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(quad_simulator src/quad_simulator.cpp)
ament_target_dependencies(quad_simulator rclcpp rclcpp_lifecycle sensor_msgs actuator_msgs rosgraph_msgs)

add_executable(quad_estimator src/quad_estimator.cpp)
ament_target_dependencies(quad_estimator rclcpp rclcpp_lifecycle sensor_msgs nav_msgs)

add_executable(quad_controller src/quad_controller.cpp)
ament_target_dependencies(quad_controller rclcpp rclcpp_lifecycle actuator_msgs nav_msgs)

install(TARGETS
  quad_simulator
  quad_estimator
  quad_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/quad_simulator.py
  scripts/quad_estimator.py
  scripts/quad_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
