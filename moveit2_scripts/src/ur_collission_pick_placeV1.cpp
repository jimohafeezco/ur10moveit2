#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

const double tau = 2 * M_PI;


class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "ur_manipulator"), //
          gripper(node, "gripper"), //gripper
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndPlace")),

          // link attacher
          node_(node)
    {
        move_group.setPoseReferenceFrame("base_link");
        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");



        
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.2);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
        gripper.move();
    }
    void pick()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);  
        move_group.allowReplanning(true);  
        move_group.setGoalTolerance(0.03); 
       

                std::vector<double> joint_group_positions_robot;
        RCLCPP_INFO(logger, "Going Home");
        // setup_joint_value_target(0,-2.5,1.5,-1.5, -1.50, -1.55);
        joint_group_positions_robot[0] = +0.0; // Shoulder Lift
        joint_group_positions_robot[1] = -2.50; // Shoulder Lift
        joint_group_positions_robot[2] = 1.50;  // Elbow
        joint_group_positions_robot[3] = -1.50; // Wrist 1
        joint_group_positions_robot[4] = -1.50; // Wrist 2
        joint_group_positions_robot[5] = +0.0; // Shoulder Lift

        move_group.setJointValueTarget(joint_group_positions_robot);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group.plan(my_plan_arm) ==
                            moveit::core::MoveItErrorCode::SUCCESS);

        if (success_arm)
            move_group.execute(my_plan_arm); 

        geometry_msgs::msg::Pose pick_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-3.14, 0, -1.57);
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = 0.86;
        pick_pose.position.y = 0.207;
        pick_pose.position.z = 0.46;
        move_group.setPoseTarget(pick_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Pick motion execution completed.");

            close_gripper();
            rclcpp::sleep_for(std::chrono::seconds(1));
            
            attachObject();
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        geometry_msgs::msg::Pose target_pose1;

        std::vector<geometry_msgs::msg::Pose> approach_waypoints;
        target_pose1.position.z -= 0.05;
        approach_waypoints.push_back(target_pose1);

        target_pose1.position.z -= 0.06;
        approach_waypoints.push_back(target_pose1);

        moveit_msgs::msg::RobotTrajectory trajectory_approach;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = move_group.computeCartesianPath(
            approach_waypoints, eef_step, jump_threshold, trajectory_approach);

        move_group.execute(trajectory_approach);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        RCLCPP_INFO(logger, "Retreat!");

        std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
        target_pose1.position.z += 0.2;
        retreat_waypoints.push_back(target_pose1);

        moveit_msgs::msg::RobotTrajectory trajectory_retreat;
        double fraction1 = move_group.computeCartesianPath(
            retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

        move_group.execute(trajectory_retreat);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        

    }

    void goHome()
    {
        // move_group.setMaxVelocityScalingFactor(1);
        // move_group.setMaxAccelerationScalingFactor(1);
        // move_group.setPlanningTime(10.0);  
        // move_group.allowReplanning(true);  
        // move_group.setGoalTolerance(0.03); 
        std::vector<double> joint_group_positions_robot;
        RCLCPP_INFO(logger, "Going Home");
        // setup_joint_value_target(0,-2.5,1.5,-1.5, -1.50, -1.55);
        joint_group_positions_robot[0] = +0.0; // Shoulder Lift
        joint_group_positions_robot[1] = -2.50; // Shoulder Lift
        joint_group_positions_robot[2] = 1.50;  // Elbow
        joint_group_positions_robot[3] = -1.50; // Wrist 1
        joint_group_positions_robot[4] = -1.50; // Wrist 2
        joint_group_positions_robot[5] = +0.0; // Shoulder Lift

        move_group.setJointValueTarget(joint_group_positions_robot);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group.plan(my_plan_arm) ==
                            moveit::core::MoveItErrorCode::SUCCESS);

        if (success_arm)
            move_group.execute(my_plan_arm); 
    }

    void place()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);  
        move_group.allowReplanning(true);  
        move_group.setGoalTolerance(0.03); 

        // Creazione della posa target per il pick
        geometry_msgs::msg::Pose place_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-3.14, 0, -1.57);
        place_pose.orientation = tf2::toMsg(orientation);
        place_pose.position.x = 0.142;
        place_pose.position.y = 0.877;
        place_pose.position.z = 0.452;

        move_group.setPoseTarget(place_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Motion execution completed.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning failed!");
        }
    }

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "cobot";  // Nome del robot in Gazebo
        request->link1_name = "wrist_3_link"; // Nome del link del gripper
        request->model2_name = "grasp_box"; // Nome dell'oggetto da afferrare
        request->link2_name = "grasp_box_base_link";    // Nome del link dell'oggetto

        while (!attach_client->wait_for_service(std::chrono::seconds(100)))
        {
            RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
        }

        auto future = attach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "cobot";  // Nome del robot in Gazebo
        request->link1_name = "wrist_3_link"; // Nome del link del gripper
        request->model2_name = "grasp_box"; // Nome dell'oggetto da afferrare
        request->link2_name = "grasp_box_base_link";    // Nome del link dell'oggetto

        while (!detach_client->wait_for_service(std::chrono::seconds(100)))
        {
            RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
        }

        auto future = detach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
        }
    }




    void addCollisionObjects()
{
        tf2::Quaternion orientation;  // orientation defaults unchanged
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.resize(5);

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

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }


private:

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    rclcpp::Logger logger;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;

};



int main(int argc, char **argv)
{
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_node");

    PickAndPlace pick_and_place(node);

    
    // Add Collision object
    pick_and_place.addCollisionObjects();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // pick_and_place.goHome();
    // Pick Execution
    pick_and_place.pick();

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Attach collision object
    pick_and_place.close_gripper();
    pick_and_place.attachObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Place Execution
    pick_and_place.place();
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Open gripper
    pick_and_place.open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(1));
    pick_and_place.detachObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // pick_and_place.detachObject();
    // rclcpp::sleep_for(std::chrono::seconds(1));


    // Spegni ROS2
    rclcpp::shutdown();
    return 0;
}