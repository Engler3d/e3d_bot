#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "e3d_bot_msgs/action/bot_task.hpp"

#include <memory>
#include <vector>

using namespace std::placeholders;

namespace arduinobot_remote
{
class E3DTaskServer : public rclcpp::Node
{
public:
  explicit E3DTaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("e3d_task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<e3d_bot_msgs::action::BotTask>(
        this, "e3d_task_server", 
        std::bind(&E3DTaskServer::goalCallback, this, _1, _2),
        std::bind(&E3DTaskServer::cancelCallback, this, _1),
        std::bind(&E3DTaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<e3d_bot_msgs::action::BotTask>::SharedPtr action_server_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const e3d_bot_msgs::action::BotTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request %d", goal->arm_joint_goal);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<e3d_bot_msgs::action::BotTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
    ///auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
    arm_move_group.stop();
    //gripper_move_group.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<e3d_bot_msgs::action::BotTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&E3DTaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<e3d_bot_msgs::action::BotTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<e3d_bot_msgs::action::BotTask::Result>();

    // MoveIt 2 Interface
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
    //auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

    auto goal = goal_handle->get_goal();

    std::vector<double> arm_joint_goal = goal->arm_joint_goal;
    //std::vector<double> gripper_joint_goal = goal->gripper_joint_goal;

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    //bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

    if (!arm_within_bounds /*|| !gripper_within_bounds*/)
    {
      RCLCPP_WARN(get_logger(), "Target joint position(s) were outside of limits, planning will clamp to the limits");
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    //moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (arm_plan_success /*&& gripper_plan_success*/)
    {
      RCLCPP_INFO(get_logger(), "Planner succeeded, moving the arm ");
      arm_move_group.move();
      //gripper_move_group.move();
      result->success = true;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "One or more planners failed!");
      result->success = false;
    }

    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal execution completed");
  }
};
}  // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::E3DTaskServer)
