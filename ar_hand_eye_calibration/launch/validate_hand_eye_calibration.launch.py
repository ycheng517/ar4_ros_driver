import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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

    calibration_aruco_publisher = Node(
        package="ar_hand_eye_calibration",
        executable="calibration_aruco_publisher.py",
        name="calibration_aruco_publisher",
        output="screen",
    )

    calibration_tf_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("easy_handeye2"),
                         "launch", "publish.launch.py")
        ]),
        launch_arguments={"name": "ar4_calibration"}.items())

    ld = LaunchDescription()
    ld.add_action(rs_launch)
    ld.add_action(aruco_recognition_node)
    ld.add_action(calibration_aruco_publisher)
    ld.add_action(calibration_tf_publisher)
    return ld
