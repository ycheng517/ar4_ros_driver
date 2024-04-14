import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    calibrate_arm = LaunchConfiguration("calibrate_arm")

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"),
                         "launch", "rs_launch.py")
        ]))

    aruco_recognition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros2_aruco"), "launch",
                         "aruco_recognition.launch.py")
        ]))
    ar_hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ar_hardware_interface"),
                         "launch", "ar_hardware.launch.py")
        ]),
        launch_arguments={
            "include_gripper": "False",
            "calibrate": calibrate_arm,
        }.items())

    calibration_args = {
        "calibration_type": "eye_on_hand",
        "robot_base_frame": "/base_link",
        "robot_effector_frame": "/link_6",
        "tracking_base_frame": "/camera_link",
        "tracking_marker_frame": "/aruco_markers",
    }

    easy_handeye2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("easy_handeye2"),
                         "launch", "calibrate.launch.py")
        ]),
        launch_arguments=calibration_args.items())

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument("calibrate_arm",
                              default_value="True",
                              description="Calibrate the arm"))
    ld.add_action(rs_launch)
    ld.add_action(aruco_recognition_launch)
    ld.add_action(ar_hardware_interface_launch)
    # ld.add_action(easy_handeye2_launch)
    return ld
