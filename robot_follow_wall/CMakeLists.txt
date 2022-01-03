cmake_minimum_required(VERSION 3.5)
project(robot_follow_wall)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(robot_follow_wall REQUIRED) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# For Message Generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindWall.srv"
  "action/OdomRecord.action"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(follow_wall src/control_robot.cpp)
ament_target_dependencies(follow_wall rclcpp geometry_msgs sensor_msgs robot_follow_wall rclcpp_action)

add_executable(find_wall_server_node src/find_wall.cpp)
ament_target_dependencies(find_wall_server_node rclcpp sensor_msgs geometry_msgs robot_follow_wall)

add_executable(record_odom_action_node src/record_odom.cpp)
ament_target_dependencies(record_odom_action_node rclcpp rclcpp_action robot_follow_wall geometry_msgs nav_msgs)

install(TARGETS
follow_wall
find_wall_server_node
record_odom_action_node
DESTINATION lib/${PROJECT_NAME}
)

install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
