#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "get_pose", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("get_pose");
  // We spin up a SingleThreadedExecutor so the program interacts with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Get current Pose
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "the reference frame is: %s", current_pose.header.frame_id.c_str());
  RCLCPP_INFO(logger, "current position x is: %f", current_pose.pose.position.x);
  RCLCPP_INFO(logger, "current position y is: %f", current_pose.pose.position.y);
  RCLCPP_INFO(logger, "current position z is: %f", current_pose.pose.position.z);
  RCLCPP_INFO(logger, "current orientation x is: %f", current_pose.pose.orientation.x);
  RCLCPP_INFO(logger, "current orientation y is: %f", current_pose.pose.orientation.y);
  RCLCPP_INFO(logger, "current orientation z is: %f", current_pose.pose.orientation.z);
  RCLCPP_INFO(logger, "current orientation w is: %f", current_pose.pose.orientation.w);

//   geometry_msgs::msg::Pose target_pose = current_pose;

//   target_pose.position.x += 0.1;

  // // Set a target Pose
  // auto const target_pose = [] {
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 1.0;
  //   msg.position.x = 0.28;
  //   msg.position.y = -0.2;
  //   msg.position.z = 0.5;
  //   return msg;
  // }();
//   move_group_interface.setPoseTarget(target_pose);

//   // Create a plan to that target pose
//   auto const [success, plan] = [&move_group_interface] {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   // Execute the plan
//   if (success)
//   {
//     move_group_interface.execute(plan);
//   }
//   else
//   {
//     RCLCPP_ERROR(logger, "Planning failed!");
//   }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}