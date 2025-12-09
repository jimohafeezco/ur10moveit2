#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <moveit_
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// class MotionTrajectorty



int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  auto move_group_gripper_interface = MoveGroupInterface(node, "gripper");

  move_group_interface.setPoseReferenceFrame("base_link");
  tf2::Quaternion zero_orientation;
  zero_orientation.setRPY(0, 0, 0);
  const geometry_msgs::msg::Quaternion zero_orientation_msg = tf2::toMsg(zero_orientation);
  geometry_msgs::msg::Pose target_pose;
  // target_pose.orientation = zero_orientation_msg;
  target_pose.position.x = 0.86;
  target_pose.position.y = 0.207;
  target_pose.position.z = 0.6;
  target_pose.orientation.x = -0.707;
  target_pose.orientation.y = 0.707;
  target_pose.orientation.z = 0.026;
  target_pose.orientation.w = 0.024;

  move_group_interface.setPoseTarget(target_pose);

    // Create identity rotation quaternion

  // Create collision object for the robot to avoid
 
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  
  collision_objects.resize(5);

  // Add the first table where the cube will originally be kept.
  collision_objects.at(0).id = "table1";
  collision_objects.at(0).header.frame_id = "world";

  // Define the primitive and its dimensions.
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.8;
  collision_objects[0].primitives[0].dimensions[2] = 1.03;

  // // Define the pose of the table.
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.3;
  collision_objects[0].primitive_poses[0].position.y = 1.12;
  collision_objects[0].primitive_poses[0].position.z = 0.515;
  collision_objects[0].primitive_poses[0].orientation = zero_orientation_msg;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "world";

  // Define the primitive and its dimensions.
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.8;
  collision_objects[1].primitives[0].dimensions[1] = 1.5;
  collision_objects[1].primitives[0].dimensions[2] = 1.03;

  // Define the pose of the table.
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.8;
  collision_objects[1].primitive_poses[0].position.y = 0.8;
  collision_objects[1].primitive_poses[0].position.z = 0.515;
  collision_objects[1].primitive_poses[0].orientation = zero_orientation_msg;

  collision_objects[1].operation = collision_objects[1].ADD;

  // // Define the object that we will be manipulating.
  collision_objects[2].header.frame_id = "world";
  collision_objects[2].id = "table3";

  // Define the primitive and its dimensions.
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.8;
  collision_objects[2].primitives[0].dimensions[1] = 1.5;
  collision_objects[2].primitives[0].dimensions[2] = 1.03;

  // Define the pose of the object.
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.8;
  collision_objects[2].primitive_poses[0].position.y = -0.72;
  collision_objects[2].primitive_poses[0].position.z = 0.515;
  collision_objects[2].primitive_poses[0].orientation = zero_orientation_msg;

  collision_objects[2].operation = collision_objects[2].ADD;



  collision_objects[3].id = "urbase";
  collision_objects[3].header.frame_id = "world";

  // Primitive
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.4;   // x
  collision_objects[3].primitives[0].dimensions[1] = 0.4;   // y
  collision_objects[3].primitives[0].dimensions[2] = 0.8;   // z

  // Pose
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.0;
  collision_objects[3].primitive_poses[0].position.y = 0.0;
  collision_objects[3].primitive_poses[0].position.z = 0.4;
  collision_objects[3].primitive_poses[0].orientation = zero_orientation_msg;
  collision_objects[3].operation = collision_objects[3].ADD;


  collision_objects[4].id = "cylinder_base";
  collision_objects[4].header.frame_id = "world";
  collision_objects[4].primitives.resize(1);
  collision_objects[4].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  collision_objects[4].primitives[0].dimensions = {0.8, 0.2}; 
  collision_objects[4].primitive_poses.resize(1);
  collision_objects[4].primitive_poses[0].position.x = 0.1;
  collision_objects[4].primitive_poses[0].position.y = -1.1;
  collision_objects[4].primitive_poses[0].position.z = 0.4;
  collision_objects[4].operation = moveit_msgs::msg::CollisionObject::ADD;



  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(collision_objects);



  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}