#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformBroadcaster


class CalibrationArucoPublisher(Node):
    """ROS2 node that listens to the aruco markers topic and publishes the 
    transform of the specific aruco marker for calibration to tf2.
    """

    def __init__(self):
        super().__init__("calibration_aruco_publisher")

        # ID of the aruco marker mounted on the robot
        self.marker_id = self.declare_parameter(
            "marker_id", 1).get_parameter_value().integer_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(ArucoMarkers,
                                                     "/aruco_markers",
                                                     self.handle_aruco_markers,
                                                     1)

    def handle_aruco_markers(self, msg: ArucoMarkers):
        cal_marker_pose = None
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id == self.marker_id:
                cal_marker_pose = msg.poses[i]
                break

        if cal_marker_pose is None:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "/camera_color_frame"
        t.child_frame_id = "/calibration_aruco"

        t.transform.translation.x = cal_marker_pose.position.x
        t.transform.translation.y = cal_marker_pose.position.y
        t.transform.translation.z = cal_marker_pose.position.z
        t.transform.rotation = cal_marker_pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = CalibrationArucoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
