#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// ---------------------------------------------------------
// Helper Functions
// ---------------------------------------------------------

bool openGripper(moveit::planning_interface::MoveGroupInterface &gripper_group) {
  RCLCPP_INFO(LOGGER, "Open Gripper!");
  gripper_group.setNamedTarget("gripper_open");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  gripper_group.execute(plan);
  return ok;
}

bool closeGripper(moveit::planning_interface::MoveGroupInterface &gripper_group) {
  RCLCPP_INFO(LOGGER, "Close Gripper!");
  gripper_group.setNamedTarget("gripper_close");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  gripper_group.execute(plan);
  return ok;
}

bool setJointPose(moveit::planning_interface::MoveGroupInterface &arm_group,
                  std::vector<double> joint_positions) {
                    
  arm_group.setJointValueTarget(joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (arm_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  arm_group.execute(plan);
  return ok;
}

bool setGoalPose(moveit::planning_interface::MoveGroupInterface &arm_group,
                 const geometry_msgs::msg::Pose &pose) {
  arm_group.setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (arm_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  arm_group.execute(plan);
  return ok;
}

// ---------------------------------------------------------
// Main Program
// ---------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;

  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
      joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
      joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // ---------------------------------------------------------
  // Home Position
  // ---------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[1] = -2.50;
  joint_group_positions_arm[2] = 1.50;
  joint_group_positions_arm[3] = -1.50;
  joint_group_positions_arm[4] = -1.55;

  setJointPose(move_group_arm, joint_group_positions_arm);

  // ---------------------------------------------------------
  // Pregrasp Pose
  // ----------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.264;

  setGoalPose(move_group_arm, target_pose1);

  // ---------------------------------------------------------
  // Open Gripper
  // ---------------------------------------------------------
  openGripper(move_group_gripper);

  // ---------------------------------------------------------
  // Approach
  // ---------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // ---------------------------------------------------------
  // Close Gripper
  // ---------------------------------------------------------
  move_group_gripper.setMaxVelocityScalingFactor(0.001);
  move_group_gripper.setMaxAccelerationScalingFactor(0.001);

  closeGripper(move_group_gripper);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // ---------------------------------------------------------
  // Retreat
  // ---------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  // ---------------------------------------------------------
  // Rotate Arm
  // ---------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
      joint_group_positions_arm);

  joint_group_positions_arm[0] = 1.57;

  setJointPose(move_group_arm, joint_group_positions_arm);

  // ---------------------------------------------------------
  // Release Object
  // ---------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Release Object!");

  openGripper(move_group_gripper);

  rclcpp::shutdown();
  return 0;
}
