#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tms_robot_interfaces/action/move_tcp_z.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp> 

class MoveTcpZActionServer : public rclcpp::Node {
public:
    using MoveTcpZ = tms_robot_interfaces::action::MoveTcpZ;
    using GoalHandleMoveTcpZ = rclcpp_action::ServerGoalHandle<MoveTcpZ>;

    explicit MoveTcpZActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("move_tcp_z_server", options), 
    move_group_interface_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](auto){}), "ur_arm")) {
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<MoveTcpZ>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "move_tcp_z",
            std::bind(&MoveTcpZActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveTcpZActionServer::handle_cancel, this, _1),
            std::bind(&MoveTcpZActionServer::handle_accepted, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Move-tcp-z action server started");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp_action::Server<MoveTcpZ>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveTcpZ::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with distance %.3f", goal->distance);
        (void)uuid;
        if (goal->distance > abs(0.050)) { // Goal reject criteria : more than 5 cm
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveTcpZ> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveTcpZ> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&MoveTcpZActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveTcpZ> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal ...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveTcpZ::Feedback>();
        auto result = std::make_shared<MoveTcpZ::Result>();

        geometry_msgs::msg::PoseStamped current_pose_stamped = move_group_interface_->getCurrentPose();

        // --- TCP - relative math using Eigen --- 
        Eigen::Isometry3d relative_move = Eigen::Isometry3d::Identity();
        relative_move.translation() << 0.0, 0.0, goal->distance;
        Eigen::Isometry3d current_eigen;
        tf2::fromMsg(current_pose_stamped.pose, current_eigen);
        Eigen::Isometry3d target_eigen = current_eigen * relative_move;
        // --- Convert back to message
        geometry_msgs::msg::Pose target_pose = tf2::toMsg(target_eigen);

        // --- Cartesian Path Planning Logic ---
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.0001; // 0.1 mm
        double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, trajectory);
        bool success = (fraction >= 0.90); // Define success: computing 90% or more of the requested path
        move_group_interface_->setMaxVelocityScalingFactor(0.1);
        move_group_interface_->setMaxAccelerationScalingFactor(0.1);
        if(success) {
            RCLCPP_INFO(this->get_logger(), "Cartesian path computed (%.2f%% covered) - Succeeded", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory = trajectory;
            moveit::core::MoveItErrorCode execute_success = move_group_interface_->execute(plan);
            if (execute_success == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Move group execute - Succeeded");
                result->outcome = true;
                goal_handle->succeed(result);
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Move group execute - Failed");
                result->outcome = false;
                goal_handle->abort(result);
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path computation failed (%.2f%% covered)", fraction * 100.0);
            result->outcome = false;
            goal_handle->abort(result);
        }
        // TODO: Publish feedback
    }

};  // class MoveTcpZActionServer

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::ExecutorOptions options;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto action_server = std::make_shared<MoveTcpZActionServer>(node_options);
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}