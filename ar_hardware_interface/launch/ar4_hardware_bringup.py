from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ar_description"), "urdf", "ar3.urdf.xacro"]
            ),
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

    # ar_hardware_interface_node = Node(
    #     package="ar_hardware_interface",
    #     executable="ar_hardware_interface_node",
    #     output="screen",
    # )

    ur_control_node = Node(
        package="ar_hardware_interface",
        executable="ar_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
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
            "10",
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
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
    )

    ld = LaunchDescription()
    # ld.add_action(ar_hardware_interface_node)
    ld.add_action(ur_control_node)
    ld.add_action(spawn_joint_controller)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster)
    return ld
