#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathFollower : public rclcpp::Node
{
public:
  PathFollower() : Node("path_follower")
  {
    // Define the planning group
    const std::string PLANNING_GROUP = "arm";
    
    // Create a MoveGroupInterface object
    auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), PLANNING_GROUP);
    
    // Define a target pose
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.5;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.5;
    target_pose.pose.orientation.w = 1.0;
    
    // Set the target pose
    move_group.setPoseTarget(target_pose.pose);
    
    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
      // Execute the planned trajectory
      move_group.move();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
