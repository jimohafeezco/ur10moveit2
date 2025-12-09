# ROS2 UR5 + Robotiq 2F-85 Simulation Setup  

![UR10 Pick and Place](pick_place_gif2.gif)


Grasping workflow with LinkAttacher

This project uses ROS2 Humble to simulate a UR10 robot with a Robotiq 2F-85 gripper. The setup includes Universal Robots simulation packages, Robotiq gripper support, and the IFRA LinkAttacher for attaching objects during grasping.

---

## Workspace Requirements

Create a ROS2 workspace. Inside the `src` folder, you’ll clone the packages listed below.


---

## Core Packages

Clone the main simulation workspace:ur10_simulation


---

## Grasping Support (LinkAttacher)

Clone this inside your `src` folder to enable simulated grasping and object attachment:

https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation

https://github.com/PickNikRobotics/ros2_robotiq_gripper



---

## Build the Workspace

```bash
cd master_ws_ros2
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
