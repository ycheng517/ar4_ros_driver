import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    calibrate_arm = LaunchConfiguration("calibrate_arm")

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"),
                         "launch", "rs_launch.py")
        ]))

    aruco_params = os.path.join(
        get_package_share_directory("ar_hand_eye_calibration"), "config",
        "aruco_parameters.yaml")
    aruco_recognition_node = Node(package='ros2_aruco',
                                  executable='aruco_node',
                                  parameters=[aruco_params])

    ar_hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ar_hardware_interface"),
                         "launch", "ar_hardware.launch.py")
        ]),
        launch_arguments={
            "include_gripper": "False",
            "calibrate": calibrate_arm,
        }.items())
    calibration_aruco_publisher = Node(
        package="ar_hand_eye_calibration",
        executable="calibration_aruco_publisher.py",
        name="calibration_aruco_publisher",
        output="screen",
    )

    calibration_args = {
        "name": "ar4_calibration",
        "calibration_type": "eye_on_base",
        "robot_base_frame": "base_link",
        "robot_effector_frame": "link_6",
        "tracking_base_frame": "camera_link",
        "tracking_marker_frame": "calibration_aruco",
    }

    easy_handeye2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("easy_handeye2"),
                         "launch", "calibrate.launch.py")
        ]),
        launch_arguments=calibration_args.items())
    easy_handeye2_delayed_launch = TimerAction(
        period=5.0,  # Delay time in seconds
        actions=[easy_handeye2_launch],
        condition=UnlessCondition(calibrate_arm))
    # If calibrating the arm, delay the launch of the handeye calibration
    # until the arm is calibrated.
    easy_handeye2_longer_delayed_launch = TimerAction(
        period=45.0,  # Delay time in seconds
        actions=[easy_handeye2_launch],
        condition=IfCondition(calibrate_arm))

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument("calibrate_arm",
                              default_value="True",
                              description="Calibrate the arm"))
    ld.add_action(rs_launch)
    ld.add_action(aruco_recognition_node)
    ld.add_action(ar_hardware_interface_launch)
    ld.add_action(calibration_aruco_publisher)
    ld.add_action(easy_handeye2_delayed_launch)
    ld.add_action(easy_handeye2_longer_delayed_launch)
    return ld
