import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
)

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Get URDF
    urdf_folder = os.path.join(get_package_share_directory("gungnir_description"), "urdf")
    urdf = os.path.join(urdf_folder, "gungnir.urdf")
    robot_description_values = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    robot_description = {'robot_description': robot_description_values}

    # Get SRDF
    srdf_folder = os.path.join(get_package_share_directory("gungnir_description"), "srdf")
    srdf = os.path.join(srdf_folder, "gungnir.srdf")
    with open(srdf, 'r') as f:
        semantic_content_values = f.read()
    semantic_content = {'robot_description_semantic': semantic_content_values}

    # Get kinematics
    robot_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "gungnir_controller",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    # Get parameters for the Servo node
    servo_yaml = load_yaml("gungnir_controller", "config/gungnir_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("gungnir_description"),
            "config/joint_limits.yaml"
        ]),
        allow_substs=True,
    )

    # Planning Configuration
    ompl_planning_yaml = load_yaml("gungnir_controller",
                                   "config/planners/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml("gungnir_controller",
                                   "config/planners/pilz_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "pilz",
        "planning_pipelines": ["ompl", "pilz"],
        "ompl": ompl_planning_yaml,
        "pilz": pilz_planning_yaml,
    }

    moveit_controller_manager = {
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("gungnir_controller"),
            "config/controllers_moveit.yaml"
        ]),
        allow_substs=True,
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            semantic_content,
            robot_kinematics,
            joint_limits,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controller_manager,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {
                "use_sim_time": False
            },
        ],
    )

    # RViz
    rviz_folder = os.path.join(
            get_package_share_directory("gungnir_controller"), "rviz")
    rviz_config = os.path.join(rviz_folder, "rviz_config.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            semantic_content,
            robot_kinematics,
        ],
    )


    ######## To do: Fix servo node and joy to servo ########
    #
    # container = ComposableNodeContainer(
    #     name="moveit_servo_demo_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::JoyToServoPub",
    #             name="controller_to_servo_node",
    #         ),
    #         ComposableNode(
    #             package="joy",
    #             plugin="joy::Joy",
    #             name="joy_node",
    #         ),
    #     ],
    #     output="screen",
    # )

    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node_main",
    #     parameters=[
    #         servo_params,
    #         robot_description,
    #         semantic_content,
    #         robot_kinematics,
    #     ],
    #     output="screen",
    # )
    #
    ###########################################################

    return LaunchDescription([
        rviz_node,
        move_group_node,
    ])