import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"),
                         "launch", "rs_launch.py")
        ]))

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

    aruco_params = os.path.join(get_package_share_directory("ar_hand_eye"),
                                "config", "aruco_parameters.yaml")
    aruco_recognition_node = Node(package='ros2_aruco',
                                  executable='aruco_node',
                                  parameters=[aruco_params])

    calibration_aruco_publisher = Node(
        package="ar_hand_eye",
        executable="calibration_aruco_publisher.py",
        name="calibration_aruco_publisher",
        output="screen",
    )

    calibration_args = {
        "name": "ar4_calibration",
        "calibration_type": "eye_on_base",
        "robot_base_frame": "base_link",
        "robot_effector_frame": "ee_link",
        "tracking_base_frame": "camera_color_frame",
        "tracking_marker_frame": "calibration_aruco",
    }

    easy_handeye2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("easy_handeye2"),
                         "launch", "calibrate.launch.py")
        ]),
        launch_arguments=calibration_args.items())

    # static transform publisher for camera_link to world
    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "camera_link"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(realsense)
    ld.add_action(static_tf_publisher)
    ld.add_action(ar_moveit)
    ld.add_action(aruco_recognition_node)
    ld.add_action(calibration_aruco_publisher)
    ld.add_action(easy_handeye2)
    return ld
