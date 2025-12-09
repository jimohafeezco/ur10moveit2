#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>


// working attach and detach taking long time
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

static rclcpp::Node::SharedPtr node_;
static moveit::planning_interface::MoveGroupInterface* arm;
static moveit::planning_interface::MoveGroupInterface* gripper;

static rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
static rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;

static constexpr double GRIP_PERCENT = 35.0;
static constexpr double EE_MIN = 0.0;
static constexpr double EE_MAX = 0.8;

// ----------------------------------------------------
// Collision Objects
// ----------------------------------------------------
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &psi)
{
    tf2::Quaternion orientation;
    std::vector<moveit_msgs::msg::CollisionObject> objs(5);

    // TABLE 1
    objs[0].id = "table1";
    objs[0].header.frame_id = "base_link";
    objs[0].primitives.resize(1);
    objs[0].primitives[0].type = objs[0].primitives[0].BOX;
    objs[0].primitives[0].dimensions = {1.5, 0.8, 1.015};
    objs[0].primitive_poses.resize(1);
    objs[0].primitive_poses[0].position.x = -0.3;
    objs[0].primitive_poses[0].position.y = 1.12;
    objs[0].primitive_poses[0].position.z = -0.3;
    objs[0].operation = objs[0].ADD;

    // TABLE 2
    objs[1].id = "table2";
    objs[1].header.frame_id = "base_link";
    objs[1].primitives.resize(1);
    objs[1].primitives[0].type = objs[1].primitives[0].BOX;
    objs[1].primitives[0].dimensions = {0.8, 1.5 ,1.015};
    objs[1].primitive_poses.resize(1);
    objs[1].primitive_poses[0].position.x = 0.8;
    objs[1].primitive_poses[0].position.y = 0.8;
    objs[1].primitive_poses[0].position.z = -0.3;
    objs[1].operation = objs[1].ADD;

    // TABLE 3
    objs[2].id = "table3";
    objs[2].header.frame_id = "base_link";
    objs[2].primitives.resize(1);
    objs[2].primitives[0].type = objs[2].primitives[0].BOX;
    objs[2].primitives[0].dimensions = {0.8, 1.5, 1.015};
    objs[2].primitive_poses.resize(1);
    objs[2].primitive_poses[0].position.x = 0.8;
    objs[2].primitive_poses[0].position.y = -0.72;
    objs[2].primitive_poses[0].position.z = -0.3;
    objs[2].operation = objs[2].ADD;

    // UR BASE
    objs[3].id = "urbase";
    objs[3].header.frame_id = "base_link";
    objs[3].primitives.resize(1);
    objs[3].primitives[0].type = objs[3].primitives[0].BOX;
    objs[3].primitives[0].dimensions = {0.4, 0.4, 0.8};
    objs[3].primitive_poses.resize(1);
    objs[3].primitive_poses[0].position.x = 0.0;
    objs[3].primitive_poses[0].position.y = 0.0;
    objs[3].primitive_poses[0].position.z = -0.4;
    objs[3].operation = objs[3].ADD;

    // CYLINDER BASE
    objs[4].id = "cylinder_base";
    objs[4].header.frame_id = "base_link";
    objs[4].primitives.resize(1);
    objs[4].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    objs[4].primitives[0].dimensions = {0.8, 0.2};
    objs[4].primitive_poses.resize(1);
    objs[4].primitive_poses[0].position.x = 0.1;
    objs[4].primitive_poses[0].position.y = -1.1;
    objs[4].primitive_poses[0].position.z = -0.4;
    objs[4].operation = objs[4].ADD;

    psi.applyCollisionObjects(objs);
}

// ----------------------------------------------------
// Simple Helpers
// ----------------------------------------------------
void goHome()
{
    RCLCPP_INFO(LOGGER, "Going Home");

    std::vector<double> home(6);
    home[0] = 0;
    home[1] = -2.50;
    home[2] = 1.50;
    home[3] = -1.50;
    home[4] = -1.50;
    home[5] = 0;

    arm->setJointValueTarget(home);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (ok) arm->execute(plan);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void openGripper()
{
    gripper->setStartStateToCurrentState();
    RCLCPP_INFO(LOGGER, "Opening gripper");
    gripper->setNamedTarget("open");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = gripper->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (ok) gripper->execute(plan);
}

void closeGripper()
{
    RCLCPP_INFO(LOGGER, "Closing gripper");

    double GP = (EE_MAX - EE_MIN) * (GRIP_PERCENT / 100.0);
    double cmd = EE_MIN + GP;

    gripper->setStartStateToCurrentState();
    gripper->setMaxVelocityScalingFactor(0.001);
    gripper->setMaxAccelerationScalingFactor(0.001);
    gripper->setNamedTarget("close_mid");
    // gripper->setJointValueTarget("robotiq_85_left_knuckle_joint", cmd);
    // gripper->move();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = gripper->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (ok) gripper->execute(plan);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void setPoseTarget(const geometry_msgs::msg::Pose &pose)
{
    arm->setPoseTarget(pose, "wrist_3_link");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (ok) arm->execute(plan);
}

void cartesianMove(const std::vector<geometry_msgs::msg::Pose> &wps)
{
    moveit_msgs::msg::RobotTrajectory traj;
    const double eef_step = 0.01;
    const double jump = 0.0;

    arm->computeCartesianPath(wps, eef_step, jump, traj);
    arm->execute(traj);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// ----------------------------------------------------
// PICK FUNCTION
// ----------------------------------------------------
void pick(const geometry_msgs::msg::Pose &pregrasp_pose,
    const std::string &model2,
    const std::string &link2)
{
    RCLCPP_INFO(LOGGER, "Starting PICK sequence");

    // 1) Move to pregrasp
    setPoseTarget(pregrasp_pose);

    // 2) Open gripper
    openGripper();

    // 3) Cartesian approach
    std::vector<geometry_msgs::msg::Pose> approach;
    geometry_msgs::msg::Pose p = pregrasp_pose;

    p.position.z -= 0.05;
    approach.push_back(p);

    p.position.z -= 0.045;
    approach.push_back(p);

    cartesianMove(approach);

    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    // 4) Close gripper
    closeGripper();

    // 5) Attach object
    {
        auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        req->model1_name = "cobot";
        req->link1_name  = "robotiq_85_left_finger_tip_link";
        req->model2_name = model2;
        req->link2_name  = link2;

        auto res = attach_client->async_send_request(req).get();
        if (!res->success) {
            RCLCPP_ERROR(LOGGER, "Attach failed: %s", res->message.c_str());
            return;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 6) Retreat
    std::vector<geometry_msgs::msg::Pose> retreat;
    p.position.z += 0.25;
    retreat.push_back(p);
    cartesianMove(retreat);
}

// ----------------------------------------------------
// PLACE FUNCTION
// ----------------------------------------------------
void place(const geometry_msgs::msg::Pose &drop_pose,
    const std::string &model2,
    const std::string &link2)
{
    RCLCPP_INFO(LOGGER, "Starting PLACE sequence");

    // 1) Move to drop position
    setPoseTarget(drop_pose);
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    // 2) Approach down
    std::vector<geometry_msgs::msg::Pose> approach2;
    geometry_msgs::msg::Pose p = drop_pose;

    p.position.z -= 0.1;
    approach2.push_back(p);

    p.position.z -= 0.1;
    approach2.push_back(p);

    cartesianMove(approach2);

    // arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    // 3) Open gripper
    openGripper();

    // 4) Detach
// 4) Detach (FAST VERSION)
    // {
    auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    req->model1_name = "cobot";
    req->link1_name  = "robotiq_85_left_finger_tip_link";
    req->model2_name = model2;
    req->link2_name  = link2;

    // Quick check the service is available (non-blocking)
    while (!detach_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(LOGGER, "Waiting for the DetachLink service...");
    }
    auto future = detach_client->async_send_request(req);

    // auto future = detach_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(LOGGER, "Object detached successfully.");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Failed to detach object.");
    }

    RCLCPP_INFO(LOGGER, "Object detached successfully.");
    // }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // rclcpp::sleep_for(std::chrono::milliseconds(80));

    // 5) Retreat up
    std::vector<geometry_msgs::msg::Pose> retreat2;
    p.position.z += 0.20;
    retreat2.push_back(p);
    cartesianMove(retreat2);
}

// ----------------------------------------------------
// MAIN
// ----------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", options);
    node_ = rclcpp::Node::make_shared("link_attacher_client_node");

    attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

    if (!attach_client->wait_for_service(std::chrono::seconds(5)) ||
        !detach_client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(LOGGER, "LinkAttacher services unavailable");
        return 1;
    }

    static const std::string ARM_GROUP = "ur_manipulator";
    static const std::string GRIPPER_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group_robot(move_group_node, ARM_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, GRIPPER_GROUP);

    arm = &move_group_robot;
    gripper = &move_group_gripper;

    move_group_robot.setPoseReferenceFrame("base_link");
    move_group_robot.setMaxAccelerationScalingFactor(1.0);
    move_group_robot.setMaxVelocityScalingFactor(1.0);
    move_group_gripper.setMaxAccelerationScalingFactor(0.001);
    move_group_gripper.setMaxVelocityScalingFactor(0.001);
    move_group_robot.setPlanningTime(10.0);
    move_group_robot.allowReplanning(true);
    move_group_robot.setGoalTolerance(0.03);

    moveit::planning_interface::PlanningSceneInterface psi;
    addCollisionObjects(psi);

    // rclcpp::executors::SingleThreadedExecutor exec;
    // exec.add_node(move_group_node);
    // exec.add_node(node_);
    // std::thread([&exec]() { exec.spin(); }).detach();

    rclcpp::executors::SingleThreadedExecutor exec;

// Only MoveIt's node goes into this executor
    exec.add_node(move_group_node);

    std::thread([&exec]() { exec.spin(); }).detach();

// node_ MUST NOT be added to executor


    // ---------------------------------------------
    // MULTI-BLOCK PICK AND PLACE
    // ---------------------------------------------

    tf2::Quaternion q;
    q.setRPY(M_PI, 0.070, -M_PI_2);

    // ------------------------
    // BLOCK 1
    // ------------------------
    geometry_msgs::msg::Pose pre1;
    pre1.position.x = 0.860;
    pre1.position.y = 0.207;
    pre1.position.z = 0.46;     // ALWAYS 0.46 pregrasp
    pre1.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Pose drop1;
    drop1.position.x = 0.20;
    drop1.position.y = 0.89;
    drop1.position.z = 0.56;
    drop1.orientation = tf2::toMsg(q);

    // ------------------------
    // BLOCK 2
    // ------------------------
    geometry_msgs::msg::Pose pre2;
    pre2.position.x = 0.75;
    pre2.position.y = 0.50;
    pre2.position.z = 0.46;     // pregrasp height ALWAYS 0.46
    pre2.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Pose drop2;
    drop2.position.x = 0.32;    // placed beside block 1
    drop2.position.y = 0.89;
    drop2.position.z = 0.56;
    drop2.orientation = tf2::toMsg(q);

    // ------------------------
    // BLOCK 3
    // ------------------------
    geometry_msgs::msg::Pose pre3;
    pre3.position.x = 0.70;
    pre3.position.y = -0.10;
    pre3.position.z = 0.46;
    pre3.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Pose drop3;
    drop3.position.x = 0.44;    // third block placement
    drop3.position.y = 0.89;
    drop3.position.z = 0.56;
    drop3.orientation = tf2::toMsg(q);

    // ------------------------
    // EXECUTION SEQUENCE
    // ------------------------

    goHome();
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    pick(pre1,"grasp_box", "grasp_box_base_link");
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();
    // goHome();
    place(drop1,"grasp_box", "grasp_box_base_link");

    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    goHome();
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    pick(pre2,"grasp_box_clone", "grasp_box_base_link");
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();
    // goHome();

    place(drop2,"grasp_box_clone", "grasp_box_base_link");
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    goHome();
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    pick(pre3, "grasp_box_clone_0", "grasp_box_base_link");
    // goHome();
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();
    place(drop3,"grasp_box_clone_0", "grasp_box_base_link");
    arm->setStartStateToCurrentState();
    gripper->setStartStateToCurrentState();

    goHome();

    rclcpp::shutdown();
    return 0;
}
