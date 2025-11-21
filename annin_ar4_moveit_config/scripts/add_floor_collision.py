#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class FloorCollisionSetup(Node):

    def __init__(self):
        super().__init__('floor_collision_setup')

        # Publisher for planning scene updates
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)

        # Wait a moment for the publisher to be ready
        self.timer = self.create_timer(2.0, self.add_floor_collision)

    def add_floor_collision(self):
        """Add a floor collision object to the planning scene"""

        # Create collision object for the floor
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = "floor"

        # Define floor as a box primitive
        floor_box = SolidPrimitive()
        floor_box.type = SolidPrimitive.BOX
        floor_box.dimensions = [0.6, 0.6, 0.01]  # 2m x 2m x 1cm floor

        # Position the floor object
        floor_pose = Pose()
        floor_pose.position.x = 0.0
        floor_pose.position.y = -0.4
        floor_pose.position.z = 0.07
        floor_pose.orientation.w = 1.0

        # Add the primitive and pose to collision object
        collision_object.primitives.append(floor_box)
        collision_object.primitive_poses.append(floor_pose)
        collision_object.operation = CollisionObject.ADD

        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.robot_state.is_diff = True
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)

        # Publish the planning scene
        self.planning_scene_pub.publish(planning_scene)

        self.get_logger().info(
            "Added floor collision object to planning scene")

        # Stop the timer after adding the floor
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    floor_setup = FloorCollisionSetup()

    try:
        for _ in range(3):
            rclpy.spin_once(floor_setup, timeout_sec=3.0)
    except KeyboardInterrupt:
        pass
    finally:
        floor_setup.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
