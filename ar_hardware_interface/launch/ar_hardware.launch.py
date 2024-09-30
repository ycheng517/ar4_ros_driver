from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    calibrate = LaunchConfiguration("calibrate")
    include_gripper = LaunchConfiguration("include_gripper")
    arduino_serial_port = LaunchConfiguration("arduino_serial_port")
    ar_model_config = LaunchConfiguration("ar_model")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ar_hardware_interface"), "urdf", "ar.urdf.xacro"
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "serial_port:=",
        serial_port,
        " ",
        "calibrate:=",
        calibrate,
        " ",
        "include_gripper:=",
        include_gripper,
        " ",
        "arduino_serial_port:=",
        arduino_serial_port,
    ])
    robot_description = {"robot_description": robot_description_content}

    joint_controllers_cfg = PathJoinSubstitution([
        FindPackageShare("ar_hardware_interface"), "config", "controllers.yaml"
    ])

    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("ar_hardware_interface"),
        "config",
        "controller_update_rate.yaml",
    ])

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

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        condition=IfCondition(include_gripper),
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
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "calibrate",
            default_value="True",
            description="Calibrate the robot on startup",
            choices=["True", "False"],
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "include_gripper",
            default_value="True",
            description="Run the servo gripper",
            choices=["True", "False"],
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "arduino_serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port of the Arduino nano for the servo gripper",
        ))
    ld.add_action(
        DeclareLaunchArgument("ar_model",
                              default_value="mk1",
                              choices=["mk1", "mk2", "mk3"],
                              description="Model of AR4"))
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_joint_controller)
    ld.add_action(gripper_controller_spawner)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster)
    return ld
