import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    ar_model_config = LaunchConfiguration("ar_model")
    ar_model_arg = DeclareLaunchArgument("ar_model",
                                         default_value="ar4",
                                         choices=["ar4", "ar4_mk3"],
                                         description="Model of AR4")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"),
                         "launch", "rs_launch.py")
        ]))

    aruco_params = os.path.join(get_package_share_directory("ar_hand_eye"),
                                "config", "aruco_parameters.yaml")
    aruco_recognition_node = Node(package='ros2_aruco',
                                  executable='aruco_node',
                                  parameters=[aruco_params])

    calibration_tf_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("easy_handeye2"),
                         "launch", "publish.launch.py")
        ]),
        launch_arguments={"name": "ar4_calibration"}.items())

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ar_description"), "urdf", "ar.urdf.xacro"]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "include_gripper:=",
        "False",
    ])
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ar_moveit_config"), "srdf", "ar.srdf.xacro"]),
        " ",
        "name:=ar",
        " ",
        "include_gripper:=",
        "False",
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "ar_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    robot_description_planning = {
        "robot_description_planning":
        load_yaml(
            "ar_moveit_config",
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
        }
    }
    ompl_planning_yaml = load_yaml("ar_moveit_config",
                                   "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ar_moveit_config", "config/controllers.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager":
        controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
        }
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    params_dict = {}
    params_dict.update(robot_description)
    params_dict.update(robot_description_semantic)
    params_dict.update(robot_description_planning)
    params_dict.update(robot_description_kinematics)
    params_dict.update(ompl_planning_pipeline_config)
    params_dict.update(
        load_yaml(
            "ar_hand_eye",
            os.path.join("config", "moveit_py_parameters.yaml"),
        ))
    params_dict.update(moveit_controllers)
    params_dict.update(trajectory_execution)
    params_dict.update(planning_scene_monitor_parameters)

    follow_aruco_node = Node(
        package="ar_hand_eye",
        executable="follow_aruco_marker.py",
        name="moveit_py",
        output="screen",
        parameters=[params_dict],
    )

    ar_moveit_launch = PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("ar_moveit_config"), "launch",
                     "ar_moveit.launch.py")
    ])
    ar_moveit_args = {
        "include_gripper": "False",
        "rviz_config_file": "moveit_with_camera.rviz"
    }.items()
    ar_moveit = IncludeLaunchDescription(ar_moveit_launch,
                                         launch_arguments=ar_moveit_args)

    return LaunchDescription([
        ar_model_arg, realsense, aruco_recognition_node,
        calibration_tf_publisher, follow_aruco_node, ar_moveit
    ])
