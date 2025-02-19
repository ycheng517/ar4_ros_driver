import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class ControllerConfigSubstitution(Substitution):
    """Substitution that fills out tf_prefix in controllers.yaml."""

    def __init__(self, file_path: Substitution, tf_prefix: Substitution):
        super().__init__()
        self._file_path = file_path
        self._tf_prefix = tf_prefix

    def perform(self, context):
        # Evaluate the file path and namespace substitutions
        file_path_val = self._file_path.perform(context)
        tf_prefix_val = self._tf_prefix.perform(context)

        with open(file_path_val, "r") as f:
            content = f.read()

        content = content.replace('$(var tf_prefix)', tf_prefix_val)

        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
        temp_file.write(content.encode("utf-8"))
        temp_file.close()
        return temp_file.name


def generate_launch_description():
    ar_model_arg = DeclareLaunchArgument("ar_model",
                                         default_value="mk3",
                                         choices=["mk1", "mk2", "mk3"],
                                         description="Model of AR4")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix_arg = DeclareLaunchArgument("tf_prefix",
                                          default_value="",
                                          description="Prefix for AR4 tf_tree")
    tf_prefix = LaunchConfiguration("tf_prefix")

    initial_joint_controllers = ControllerConfigSubstitution(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_driver"), "config", "controllers.yaml"
        ]),
        tf_prefix=tf_prefix)

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_description"),
            "urdf",
            "ar_gazebo.urdf.xacro",
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "simulation_controllers:=",
        initial_joint_controllers,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    gripper_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    # Gazebo nodes
    world = os.path.join(get_package_share_directory('annin_ar4_gazebo'),
                         'worlds', 'empty.world')

    # Bridge
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output='screen')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={
            'gz_args':
            f'-r -v 4 --physics-engine gz-physics-bullet-featherstone-plugin {world}',
            'on_exit_shutdown': 'True'
        }.items())

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", ar_model_config, "-topic", "robot_description"],
        output="screen",
    )

    return LaunchDescription([
        ar_model_arg,
        tf_prefix_arg,
        gazebo_bridge,
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        gripper_joint_controller_spawner_started,
    ])
