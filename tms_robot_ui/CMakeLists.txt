cmake_minimum_required(VERSION 3.8)
project(tms_robot_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Xml)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tms_robot_interfaces REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)

include_directories(include)

add_executable(tms_robot_ui
  src/main.cpp
  src/main_window.cpp
  src/ros_bridge.cpp
  src/rviz_widget.cpp
  include/tms_robot_ui/main_window.hpp
  include/tms_robot_ui/ros_bridge.hpp
  include/tms_robot_ui/rviz_widget.hpp
)

target_include_directories(tms_robot_ui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(tms_robot_ui
  Qt5::Widgets
  Qt5::Xml
)

ament_target_dependencies(tms_robot_ui
  ament_index_cpp
  rclcpp
  rclcpp_action
  tms_robot_interfaces
  rviz_ogre_vendor
  rviz_common
  rviz_rendering
  rviz_default_plugins
)

install(TARGETS tms_robot_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()