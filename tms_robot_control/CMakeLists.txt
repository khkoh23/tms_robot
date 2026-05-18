cmake_minimum_required(VERSION 3.8)
project(tms_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
find_package(tms_robot_interfaces REQUIRED)

include_directories(include)

# add_executable(move_tcp_z_server src/move_tcp_z_server.cpp)
# ament_target_dependencies(move_tcp_z_server 
#   Eigen3
#   geometry_msgs 
#   moveit_ros_planning_interface 
#   rclcpp 
#   rclcpp_action
#   tf2_eigen
#   tf2_geometry_msgs
#   tms_robot_interfaces)

# add_executable(tms_robot_executor src/tms_robot_executor.cpp)
# ament_target_dependencies(tms_robot_executor 
#   behaviortree_cpp 
#   geometry_msgs 
#   rclcpp
#   rclcpp_action
#   tms_robot_interfaces)

set(MOVEIT_HELPER_SOURCES
  src/moveit/moveit_context.cpp
)

set(BT_NODE_SOURCES
  src/bt_nodes/check_system_ready.cpp
  src/bt_nodes/move_arm_named_target.cpp
  src/bt_nodes/move_to_frame_offset_pose.cpp
  src/bt_nodes/report_status.cpp
  src/bt_nodes/verify_named_target_reached.cpp
  src/bt_nodes/wait_node.cpp
)

add_executable(robot_task_executor 
  src/main.cpp
  src/task_executor_node.cpp
  ${BT_NODE_SOURCES}
  ${MOVEIT_HELPER_SOURCES}
)

target_include_directories(robot_task_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_task_executor
  ament_index_cpp
  behaviortree_cpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tms_robot_interfaces
)

install(TARGETS 
  # move_tcp_z_server
  # tms_robot_executor
  robot_task_executor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  tree
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
