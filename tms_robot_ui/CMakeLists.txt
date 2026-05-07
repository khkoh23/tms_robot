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
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Xml)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_ft_sensor_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tms_robot_interfaces REQUIRED)

include_directories(include)

set(CMAKE_AUTOUIC_SEARCH_PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/resource
)

add_executable(tms_robot_ui
  src/main.cpp
  src/main_window.cpp
  src/ros_bridge.cpp
  src/rviz_widget.cpp
  src/camera_view_widget.cpp 
  resource/main_window.ui
  include/tms_robot_ui/main_window.hpp
  include/tms_robot_ui/ros_bridge.hpp
  include/tms_robot_ui/rviz_widget.hpp
  include/tms_robot_ui/camera_view_widget.hpp 
)

target_include_directories(tms_robot_ui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(tms_robot_ui
  Qt5::Widgets
  Qt5::Xml
  cv_bridge::cv_bridge
)

ament_target_dependencies(tms_robot_ui
  ament_index_cpp
  cv_bridge
  geometry_msgs
  image_transport
  OpenCV
  rclcpp
  rclcpp_action
  robotiq_ft_sensor_interfaces
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  sensor_msgs
  std_msgs
  tms_robot_interfaces
)

install(TARGETS tms_robot_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()