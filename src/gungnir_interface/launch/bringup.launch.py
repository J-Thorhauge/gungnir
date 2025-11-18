import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf_folder = os.path.join(get_package_share_directory("gungnir_description"), "urdf")
    urdf = os.path.join(urdf_folder, "gungnir.urdf")
    robot_description_values = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    robot_description = {'robot_description': robot_description_values}

    controller_config = PathJoinSubstitution([
        FindPackageShare("gungnir_interface"), 
        "config", 
        "controllers.yaml",
    ])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_config,
        ],
        output="screen",
    )

    spawn_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "1000",
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "1000",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "argument_name",
            default_value="some_value",
            description="This is an example argument.",
        ))
    
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_joint_controller)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster)
    return ld
