#pragma once

#include <future>
#include <map>
#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/rclcpp.hpp>

class MoveItContext {
public:
  enum class MotionStatus {
    RUNNING, SUCCESS, FAILURE
  };
  explicit MoveItContext(rclcpp::Node::SharedPtr node);
  bool checkSystemReady(const std::string & planning_group, const std::string & target, std::string & error_msg);
  bool startMoveToNamedTarget(const std::string & planning_group, const std::string & target, std::string & error_msg);
  bool startMoveToFrameOffsetPose(const std::string & planning_group, 
    const std::string & tcp_link, 
    const std::string & reference_frame, 
    double x, 
    double y, 
    double z, 
    double roll, 
    double pitch, 
    double yaw, 
    double velocity_scale, 
    double acceleration_scale, 
    std::string & error_msg);
  MotionStatus pollMotionResult(std::string & error_msg);
  void stopMotion();
  bool verifyNamedTargetReached(const std::string & planning_group, 
    const std::string & target,
    double tolerance,
    double & max_error,
    std::string & worst_joint,
    std::string & error_msg);
  bool startMoveToPoseStamped(const std::string & planning_group,
    const std::string & tcp_link, 
    const geometry_msgs::msg::PoseStamped & target_pose, 
    double velocity_scale, 
    double acceleration_scale, 
    double planning_time, 
    int planning_attempts, 
    double position_tolerance, 
    double orientation_tolerance, 
    std::string & error_msg);
  bool startMoveTcpRelativeZ(const std::string & planning_group,
    const std::string & tcp_link,
    double distance_m,
    double velocity_scale,
    double acceleration_scale,
    double eef_step,
    double min_fraction,
    std::string & error_msg);

private:
  moveit::planning_interface::MoveGroupInterface * getMoveGroup(const std::string & planning_group, std::string & error_msg);
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>> move_groups_;
  moveit::planning_interface::MoveGroupInterface::Plan active_plan_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
  std::string active_planning_group_;
};