cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(ros2_linkattacher REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Generate the executable
add_executable(joint_space_trajectory_exec
    src/double_motion_pose.cpp)
target_include_directories(joint_space_trajectory_exec
    PUBLIC include)
ament_target_dependencies(joint_space_trajectory_exec
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

# Generate the executable
add_executable(goal_pose_trajectory
    src/goal_pose_trajectory.cpp)
target_include_directories(goal_pose_trajectory
    PUBLIC include)
ament_target_dependencies(goal_pose_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
    

# Generate the executable
add_executable(kinematics_plugin_trajectory
    src/kinematics_plugin_trajectory.cpp)
target_include_directories(kinematics_plugin_trajectory
    PUBLIC include)
ament_target_dependencies(kinematics_plugin_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

# Generate the executable
add_executable(ur_moveit_objects
    src/ur_moveit_objects.cpp)
target_include_directories(ur_moveit_objects
    PUBLIC include)
ament_target_dependencies(ur_moveit_objects
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    sensor_msgs
    geometry_msgs
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)


# Generate the executable
add_executable(ur_goal_pose_trajectory
    src/ur_goal_pose_trajectory.cpp)
target_include_directories(ur_goal_pose_trajectory
    PUBLIC include)
ament_target_dependencies(ur_goal_pose_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    sensor_msgs
    geometry_msgs
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)


# Generate the executable
add_executable(ur_ik_collission
    src/ur_ik_collission.cpp)
target_include_directories(ur_ik_collission
    PUBLIC include)
ament_target_dependencies(ur_ik_collission
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    sensor_msgs
    geometry_msgs
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

# Generate the executable
add_executable(ur_ik_collission_pick_place
    src/ur_ik_collission_pick_place.cpp)
target_include_directories(ur_ik_collission_pick_place
    PUBLIC include)
ament_target_dependencies(ur_ik_collission_pick_place
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    sensor_msgs
    ros2_linkattacher
    linkattacher_msgs
    geometry_msgs
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)



# Generate the executable
add_executable(ur_collission_pick_placeV1
    src/ur_collission_pick_placeV1.cpp)
target_include_directories(ur_collission_pick_placeV1
    PUBLIC include)
ament_target_dependencies(ur_collission_pick_placeV1
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    sensor_msgs
    ros2_linkattacher
    linkattacher_msgs
    geometry_msgs
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)



# Generate the executable
add_executable(ur_ik_collission_pick_place_attach
    src/ur_ik_collission_pick_place_attach.cpp)
target_include_directories(ur_ik_collission_pick_place_attach
    PUBLIC include)
ament_target_dependencies(ur_ik_collission_pick_place_attach
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    sensor_msgs
    ros2_linkattacher
    linkattacher_msgs
    geometry_msgs
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)


# Generate the executable
add_executable(UR10_Pick_Place
    src/UR10_Pick_Place.cpp)
target_include_directories(UR10_Pick_Place
    PUBLIC include)
ament_target_dependencies(UR10_Pick_Place
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    sensor_msgs
    ros2_linkattacher
    linkattacher_msgs
    geometry_msgs
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
    
# Install the executable
install(TARGETS 
    joint_space_trajectory_exec
    UR10_Pick_Place
    goal_pose_trajectory
    ur_goal_pose_trajectory
    ur_moveit_objects
    ur_ik_collission
    ur_ik_collission_pick_place
    ur_collission_pick_placeV1
    ur_ik_collission_pick_place_attach
    kinematics_plugin_trajectory
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
