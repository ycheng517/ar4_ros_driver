from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    calibrate = LaunchConfiguration("calibrate")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ar_hardware_interface"), "urdf", "ar.urdf.xacro"]
            ),
            " ",
            "name:=ar",
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "calibrate:=",
            calibrate,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    joint_controllers_cfg = PathJoinSubstitution(
        [FindPackageShare("ar_hardware_interface"), "config", "controllers.yaml"]
    )

    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ar_hardware_interface"),
            "config",
            "controller_update_rate.yaml",
        ]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            robot_description,
            ParameterFile(joint_controllers_cfg, allow_substs=True),
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
            "60",
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
            "60",
        ],
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port to connect to the robot",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "calibrate",
            default_value="True",
            description="Calibrate the robot on startup",
            choices=["True", "False"],
        )
    )
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_joint_controller)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster)
    return ld
