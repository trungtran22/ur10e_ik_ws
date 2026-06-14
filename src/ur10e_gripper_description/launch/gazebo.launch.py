from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("ur10e_gripper_description")
    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "ur10e_with_gripper.urdf.xacro"])
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file]),
        "use_sim_time": True,
    }

    # 1) Gazebo Classic (server + client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ]))

    # 2) robot_description + TF
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description])

    # 3) spawn robot into Gazebo from topic robot_description
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
                 arguments=["-topic", "robot_description", "-entity", "ur10e_with_gripper"],
                 output="screen")

    # 4) spawner controllers
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["joint_trajectory_controller", "-c", "/controller_manager"])
    grip = Node(package="controller_manager", executable="spawner",
                arguments=["gripper_controller", "-c", "/controller_manager"])

    # 5) node IK 
    ik = Node(package="ur10e_ik_control", executable="ik_control_node",
              output="screen", parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        gazebo, rsp, spawn,
        # xâu chuỗi: spawn xong -> jsb; jsb xong -> arm + grip + node IK
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, grip, ik])),
    ])