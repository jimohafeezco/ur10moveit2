#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

const double tau = 2 * M_PI;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static rclcpp::Node::SharedPtr node_;
static rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
static rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;
// static rclcpp::Logger logger = rclcpp::get_logger("move_group_demo");
static rclcpp::Logger logger = rclcpp::get_logger("link_attacher");
static constexpr double GRIP_PERCENT = 50.0;    // close to 70 %
static constexpr double EE_MIN       = 0.0;     // fully open
static constexpr double EE_MAX       = 0.8;     // fully closed
// ----------------------------------------------
// ADD COLLISION OBJECTS FUNCTION
// ----------------------------------------------

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    tf2::Quaternion orientation;  // orientation defaults unchanged
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(6);

    // TABLE 1
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "world";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions = {1.5, 0.8, 0.75};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = -0.3;
    collision_objects[0].primitive_poses[0].position.y = 1.12;
    collision_objects[0].primitive_poses[0].position.z = 0.515;
    collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(orientation);
    collision_objects[0].operation = collision_objects[0].ADD;

    // TABLE 2
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "world";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions = {0.8, 1.5,0.75};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.8;
    collision_objects[1].primitive_poses[0].position.y = 0.8;
    collision_objects[1].primitive_poses[0].position.z = 0.515;
    collision_objects[1].primitive_poses[0].orientation = tf2::toMsg(orientation);
    collision_objects[1].operation = collision_objects[1].ADD;

    // TABLE 3
    collision_objects[2].id = "table3";
    collision_objects[2].header.frame_id = "world";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions = {0.8, 1.5, 0.80};
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.8;
    collision_objects[2].primitive_poses[0].position.y = -0.72;
    collision_objects[2].primitive_poses[0].position.z = 0.515;
    collision_objects[2].primitive_poses[0].orientation = tf2::toMsg(orientation);
    collision_objects[2].operation = collision_objects[2].ADD;

    // UR BASE BOX
    collision_objects[3].id = "urbase";
    collision_objects[3].header.frame_id = "world";
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions = {0.4, 0.4, 0.8};
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.0;
    collision_objects[3].primitive_poses[0].position.y = 0.0;
    collision_objects[3].primitive_poses[0].position.z = 0.4;
    collision_objects[3].primitive_poses[0].orientation = tf2::toMsg(orientation);
    collision_objects[3].operation = collision_objects[3].ADD;

    // CYLINDER BASE
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
    // TABLE 3

    collision_objects[5].id = "cubicle";
    collision_objects[5].header.frame_id = "world";
    collision_objects[5].primitives.resize(1);
    collision_objects[5].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[5].primitives[0].dimensions = {1.35, 1.5, 0.80};
    collision_objects[5].primitive_poses.resize(1);
    collision_objects[5].primitive_poses[0].position.x = -1.5;
    collision_objects[5].primitive_poses[0].position.y = -0.053;
    collision_objects[5].primitive_poses[0].position.z = 0.4;
    collision_objects[5].primitive_poses[0].orientation = tf2::toMsg(orientation);
    collision_objects[5].operation = collision_objects[2].ADD;

    // WALL PARAMETERS
    std::vector<moveit_msgs::msg::CollisionObject> walls;

    double wall_thickness = 0.05;
    double wall_height = 0.80;

    double cx = -1.5;
    double cy = -0.053;
    double cz = 0.4;

    double base_w = 1.35;
    double base_d = 1.50;
    double base_h = 0.80;

    double half_w = base_w / 2.0;   // 0.675
    double half_d = base_d / 2.0;   // 0.75

    double base_top_z = cz + base_h/2.0;     // 0.80
    double wall_center_z = base_top_z + wall_height/2.0;  // 1.20


    // --------------------------------------------------
    // BACK WALL (spans width, sits at rear edge)
    // --------------------------------------------------
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.id = "cubicle_left_wall";
        obj.header.frame_id = "world";

        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        obj.primitives[0].dimensions = {
            base_w,          // full width
            wall_thickness,  // thickness
            wall_height      // height
        };

        obj.primitive_poses.resize(1);
        obj.primitive_poses[0].position.x = cx;
        obj.primitive_poses[0].position.y = cy - half_d;  // left edge
        obj.primitive_poses[0].position.z = wall_center_z;

        tf2::Quaternion q;
        q.setRPY(0,0,0);
        obj.primitive_poses[0].orientation = tf2::toMsg(q);

        obj.operation = obj.ADD;
        walls.push_back(obj);
    }

    {
        moveit_msgs::msg::CollisionObject obj;
        obj.id = "cubicle_right_wall";
        obj.header.frame_id = "world";

        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        obj.primitives[0].dimensions = {
            base_w,          // full width
            wall_thickness,  // thickness
            wall_height      // height
        };

        obj.primitive_poses.resize(1);
        obj.primitive_poses[0].position.x = cx;
        obj.primitive_poses[0].position.y = cy+ half_d ;  // right edge
        obj.primitive_poses[0].position.z = wall_center_z;

        tf2::Quaternion q;
        q.setRPY(0,0,0);
        obj.primitive_poses[0].orientation = tf2::toMsg(q);

        obj.operation = obj.ADD;
        walls.push_back(obj);
    }
    // --------------------------------------------------
    // LEFT WALL (runs depth direction)
    // --------------------------------------------------
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.id = "cubicle_front_wall";
        obj.header.frame_id = "world";

        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        obj.primitives[0].dimensions = {
            wall_thickness,  // thickness
            base_d,          // full depth
            wall_height
        };

        obj.primitive_poses.resize(1);
        obj.primitive_poses[0].position.x = cx - half_w;   // front edge
        obj.primitive_poses[0].position.y = cy;
        obj.primitive_poses[0].position.z = wall_center_z;

        tf2::Quaternion q;
        q.setRPY(0,0,0);
        obj.primitive_poses[0].orientation = tf2::toMsg(q);

        obj.operation = obj.ADD;
        walls.push_back(obj);
    }


    // --------------------------------------------------
    // // RIGHT WALL (runs depth direction)
    // // --------------------------------------------------
    // {
    //     moveit_msgs::msg::CollisionObject obj;
    //     obj.id = "cubicle_back_wall";
    //     obj.header.frame_id = "world";

    //     obj.primitives.resize(1);
    //     obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    //     obj.primitives[0].dimensions = {
    //         wall_thickness,  // thickness
    //         base_d,          // full depth
    //         wall_height
    //     };

    //     obj.primitive_poses.resize(1);
    //     obj.primitive_poses[0].position.x = cx + half_w;   // back edge
    //     obj.primitive_poses[0].position.y = cy;
    //     obj.primitive_poses[0].position.z = wall_center_z;

    //     tf2::Quaternion q;
    //     q.setRPY(0,0,0);
    //     obj.primitive_poses[0].orientation = tf2::toMsg(q);

    //     obj.operation = obj.ADD;
    //     walls.push_back(obj);
    // }


    // --------------------------------------------------
    // APPLY TO SCENE
    // --------------------------------------------------
    planning_scene_interface.applyCollisionObjects(walls);


    planning_scene_interface.applyCollisionObjects(collision_objects);
}
void attachObject() {
    auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();

    request->model1_name = "cobot";
    request->link1_name  = "wrist_3_link";
    request->model2_name = "grasp_box";
    request->link2_name = "grasp_box_base_link";    // Nome del link dell'oggetto

    while (!attach_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
    }

    auto future = attach_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(logger, "Object attached.");
    }
    else {
        RCLCPP_ERROR(logger, "Attach failed.");
    }
}



bool plan_success_gripper_ = false;
float delta_ = 0.040; // meters
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::Logger logger;
    rclcpp::NodeOptions node_options;
    rclcpp::Node::SharedPtr node;

    // auto node = std::make_shared<rclcpp::Node>("ur_robot_pick_place");

    // ros::init(argc, argv, "ur_robot_pick_place");
    // ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
              // link attacher


    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node =
        rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    // auto logger = rclcpp::get_logger("move_group_interface_tutorial");

    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    // rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;

    // auto node_ = rclcpp::Node::make_shared("link_attacher_client_node");
    node_ = rclcpp::Node::make_shared("link_attacher_client_node");
    auto attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    auto detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    if (!attach_client->wait_for_service(std::chrono::seconds(5)) || !detach_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(LOGGER, "LinkAttacher services unavailable");
        return 1;
    }
    executor.add_node(move_group_node);
    executor.add_node(node_);

    std::thread([&executor]() { executor.spin(); }).detach();
    // attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    // detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";


    moveit::planning_interface::MoveGroupInterface move_group_robot(
        move_group_node, PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_gripper(
        move_group_node, PLANNING_GROUP_GRIPPER);


    move_group_robot.setPoseReferenceFrame("base_link");
    move_group_robot.setPlanningTime(10.0);
    move_group_robot.allowReplanning(true);  
    move_group_robot.setGoalTolerance(0.03); 
    move_group_robot.setPlanningTime(10.0);  


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // ADD COLLISIONS
    addCollisionObjects(planning_scene_interface);

    const moveit::core::JointModelGroup *joint_model_group_arm =
    move_group_robot.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    const moveit::core::JointModelGroup *joint_model_group_gripper =
    move_group_gripper.getCurrentState()->getJointModelGroup(
        PLANNING_GROUP_GRIPPER);

  // Get Current State
    moveit::core::RobotStatePtr current_state_arm =
        move_group_robot.getCurrentState(10);
    moveit::core::RobotStatePtr current_state_gripper =
        move_group_gripper.getCurrentState(10);

    std::vector<double> joint_group_positions_gripper;
    std::vector<double> joint_group_positions_robot;
    
    current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                joint_group_positions_robot);
    current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                    joint_group_positions_gripper);

    move_group_robot.setStartStateToCurrentState();
    move_group_gripper.setStartStateToCurrentState();

    move_group_robot.setMaxVelocityScalingFactor(1.0);
    move_group_robot.setMaxAccelerationScalingFactor(1.0);
    // Go Home
    RCLCPP_INFO(LOGGER, "Going Home");
    // setup_joint_value_target(0,-2.5,1.5,-1.5, -1.50, -1.55);
    joint_group_positions_robot[0] = 0; // Shoulder Lift
    joint_group_positions_robot[1] = -2.50; // Shoulder Lift
    joint_group_positions_robot[2] = 1.50;  // Elbow
    joint_group_positions_robot[3] = -1.50; // Wrist 1
    joint_group_positions_robot[4] = -1.50; // Wrist 2
    joint_group_positions_robot[5] = 0; // Shoulder Lift

    move_group_robot.setJointValueTarget(joint_group_positions_robot);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success_arm = (move_group_robot.plan(my_plan_arm) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

    if (success_arm)
        move_group_robot.execute(my_plan_arm); 

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0.070, -M_PI_2);
    // orientation.setRPY(0, 0, M_PI / 2);


  // Pregrasp
    RCLCPP_INFO(LOGGER, "Pregrasp Position");
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.8602;
    target_pose1.position.y = 0.207;
    target_pose1.position.z = 0.46;
    target_pose1.orientation = tf2::toMsg(orientation);
    // target_pose1.orientation.x = 0.707;
    // target_pose1.orientation.y = -0.707;
    // target_pose1.orientation.z = -0.026;
    // target_pose1.orientation.w = -0.025;
    move_group_robot.setPoseTarget(target_pose1, "wrist_3_link");
    // move_group_robot.setPathConstraints()

    success_arm = (move_group_robot.plan(my_plan_arm) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (success_arm)
        move_group_robot.execute(my_plan_arm);
    // move_group_robot.execute(my_plan_arm);
    
  // Open Gripper

    RCLCPP_INFO(LOGGER, "Open Gripper!");

    move_group_gripper.setNamedTarget("open");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                            moveit::core::MoveItErrorCode::SUCCESS);
    if (success_gripper)
        move_group_gripper.execute(my_plan_gripper);
    move_group_gripper.setStartStateToCurrentState();
    move_group_robot.setStartStateToCurrentState();

    // Approach
    RCLCPP_INFO(LOGGER, "Approach to object!");

    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    target_pose1.position.z -= 0.05;
    approach_waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.045;
    approach_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group_robot.computeCartesianPath(
        approach_waypoints, eef_step, jump_threshold, trajectory_approach);

    move_group_robot.execute(trajectory_approach);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Update MoveIt robot state AFTER the Cartesian motion
    move_group_robot.setStartStateToCurrentState();
    move_group_gripper.setStartStateToCurrentState();

    //   Close Gripper
    // move_group_gripper.setMaxVelocityScalingFactor(0.01); // For the gripper
    // move_group_gripper.setMaxAccelerationScalingFactor(0.01); // For the gripper
    // move_group_robot.setMaxVelocityScalingFactor(0.01);
    // move_group_robot.setMaxAccelerationScalingFactor(0.01);
    RCLCPP_INFO(LOGGER, "Close Gripper!!");
    move_group_gripper.setStartStateToCurrentState();
    move_group_robot.setStartStateToCurrentState();
    move_group_gripper.setMaxVelocityScalingFactor(0.001);
    move_group_gripper.setMaxAccelerationScalingFactor(0.001);

  // 4) Close gripper to GRIP_PERCENT
  {
    double GP  = (EE_MAX - EE_MIN) * (GRIP_PERCENT/100.0);
    double cmd = EE_MIN + GP;
    RCLCPP_INFO(LOGGER, "Closing gripper to %.1f%% -> %.3f rad", GRIP_PERCENT, cmd);
    move_group_gripper.setStartStateToCurrentState();
    move_group_gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", cmd);
    move_group_gripper.move();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
    // move_group_gripper.setNamedTarget("close_mid");

    // joint_group_positions_gripper[0] = 0.76;
    // move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
    // success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
    //                     moveit::core::MoveItErrorCode::SUCCESS);

    // move_group_gripper.execute(my_plan_gripper);
    // // attachObject();
    // // rclcpp::sleep_for(std::chrono::seconds(1));
    // move_group_gripper.setStartStateToCurrentState();
    // move_group_robot.setStartStateToCurrentState();
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 5) Attach at finger tip
  {
    auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    req->model1_name = "cobot";
    req->link1_name  = "robotiq_85_left_finger_tip_link";
    req->model2_name = "grasp_box";
    req->link2_name  = "grasp_box_base_link";
    while (!attach_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
    }
    auto res = attach_client->async_send_request(req).get();
    if (!res->success) {
      RCLCPP_ERROR(LOGGER, "Attach failed: %s", res->message.c_str());
      return 1;
    }
  }





    RCLCPP_INFO(LOGGER, "Retreat!");

    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    target_pose1.position.z += 0.25;
    retreat_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat;
    double fraction1 = move_group_robot.computeCartesianPath(
        retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

    move_group_robot.execute(trajectory_retreat);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    

    // move_group_gripper.setStartStateToCurrentState();
    // move_group_robot.setStartStateToCurrentState();

    // RCLCPP_INFO(LOGGER, "Rotating Arm");

    RCLCPP_INFO(LOGGER, "Drop Position");
    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = 0.2;
    target_pose2.position.y = 0.89;
    target_pose2.position.z = 0.56;
    target_pose2.orientation = tf2::toMsg(orientation);
    // 0.995, -0.089, -0.034, -0.011
    // target_pose2.orientation.x = 0.995;
    // target_pose2.orientation.y = -0.089;
    // target_pose2.orientation.z = -0.034;
    // target_pose2.orientation.w = -0.011;
    move_group_robot.setPoseTarget(target_pose2, "wrist_3_link");
    // move_group_robot.setPathConstraints()

    success_arm = (move_group_robot.plan(my_plan_arm) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (success_arm)
        move_group_robot.execute(my_plan_arm);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    move_group_gripper.setStartStateToCurrentState();
    move_group_robot.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> approach_waypoints2;
    target_pose2.position.z -= 0.1;
    approach_waypoints2.push_back(target_pose2);
    target_pose2.position.z -= 0.1;
    approach_waypoints2.push_back(target_pose2);
    moveit_msgs::msg::RobotTrajectory trajectory_approach2;


    double fraction2 = move_group_robot.computeCartesianPath(
        approach_waypoints2, eef_step, jump_threshold, trajectory_approach2);

    move_group_robot.execute(trajectory_approach2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    move_group_gripper.setStartStateToCurrentState();
    move_group_robot.setStartStateToCurrentState();
    move_group_gripper.setMaxVelocityScalingFactor(0.001);
    move_group_gripper.setMaxAccelerationScalingFactor(0.001);

    RCLCPP_INFO(LOGGER, "Open Gripper");

    move_group_gripper.setNamedTarget("open");



    success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

    move_group_gripper.execute(my_plan_gripper);

      // 9) Detach
  {
    auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    req->model1_name = "cobot";
    req->link1_name  = "robotiq_85_left_finger_tip_link";
    req->model2_name = "grasp_box";
    req->link2_name  = "grasp_box_base_link";
    auto res = detach_client->async_send_request(req).get();
    if (!res->success) {
      RCLCPP_ERROR(LOGGER, "Detach failed: %s", res->message.c_str());
      return 1;
    }
  }

    move_group_gripper.setStartStateToCurrentState();
    move_group_robot.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> retreat_waypoints2;
    target_pose2.position.z += 0.2;
    retreat_waypoints2.push_back(target_pose2);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat2;


    double fraction3 = move_group_robot.computeCartesianPath(
        retreat_waypoints2, eef_step, jump_threshold, trajectory_retreat2);

    move_group_robot.execute(trajectory_retreat2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


    move_group_gripper.setStartStateToCurrentState();
    move_group_robot.setStartStateToCurrentState();


    RCLCPP_INFO(LOGGER, "Going back up");
    joint_group_positions_robot[0] = 0; // Shoulder Lift
    joint_group_positions_robot[1] = -2.50; // Shoulder Lift
    joint_group_positions_robot[2] = 1.50;  // Elbow
    joint_group_positions_robot[3] = -1.50; // Wrist 1
    joint_group_positions_robot[4] = -1.50; // Wrist 2
    joint_group_positions_robot[5] = 0; // Shoulder Lift

    move_group_robot.setJointValueTarget(joint_group_positions_robot);

    bool success_arm_home = (move_group_robot.plan(my_plan_arm) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

    if (success_arm_home)
        move_group_robot.execute(my_plan_arm); 
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO(LOGGER, "Close Gripper!!");

    move_group_gripper.setNamedTarget("close");
    success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

    move_group_gripper.execute(my_plan_gripper);
    move_group_gripper.setStartStateToCurrentState();
    move_group_robot.setStartStateToCurrentState();
    // ros::shutdown();
    rclcpp::shutdown();

    return 0;
}
