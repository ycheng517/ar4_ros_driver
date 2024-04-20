#!/usr/bin/env python3
"""
A script to follow an aruco marker with a robot arm using MoveItPy.
"""

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.time import Time

# moveit python library
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from moveit.planning import MoveItPy
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_geometry_msgs import do_transform_pose


class ArucoMarkerFollower(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("aruco_marker_follower")
        self.logger = self.get_logger()
        # self.get_all_params()
        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")

        self._last_marker_pose = None
        # ID of the aruco marker mounted on the robot
        self.marker_id = self.declare_parameter(
            "marker_id", 1).get_parameter_value().integer_value

        self.subscription = self.create_subscription(ArucoMarkers,
                                                     "/aruco_markers",
                                                     self.handle_aruco_markers,
                                                     1)
        self.run_timer = self.create_timer(5.0, self.run)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_all_params(self):
        params = self.get_parameters(
            ["/robot_description", "/robot_description_semantic"])
        param_dict = {}
        for param in params:
            param_dict[param.name] = param.value
        print(param_dict)

    def handle_aruco_markers(self, msg: ArucoMarkers):
        cal_marker_pose = None
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id == self.marker_id:
                cal_marker_pose = msg.poses[i]
                break

        if cal_marker_pose is None:
            return

        self._last_marker_pose = cal_marker_pose

    def run(self):
        if self._last_marker_pose is None:
            return

        # get pose in robot base frame
        transformed_pose = self.transform_pose(self._last_marker_pose,
                                               "camera_link", "base_link")
        self.logger.info(
            f"Following marker at position: {transformed_pose.position}, " + \
            f"orientation: {transformed_pose.orientation}"
        )
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose = transformed_pose

        # set plan start state to current state
        self.arm.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")

        # plan to goal
        self._plan_and_execute()

    def transform_pose(self, pose: Pose, source_frame,
                       target_frame: str) -> Pose:
        # first flip the pose in x axis by 180 degrees
        flip_transform = TransformStamped()
        flip_transform.transform.rotation.w = 0.0
        flip_transform.transform.rotation.x = 1.0
        flipped_pose = do_transform_pose(pose, flip_transform)
        pose.orientation = flipped_pose.orientation

        # Get the transform from source frame to target frame
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                    Time())

        # Transform the pose
        transformed_pose = do_transform_pose(pose, transform)

        return transformed_pose

    def _plan_and_execute(self):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
        plan_result = self.arm.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="moveit_py")
    node = ArucoMarkerFollower(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
