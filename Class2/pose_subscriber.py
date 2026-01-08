#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')
        self.subscription = self.create_subscription(
            Pose,
            "turtle1/pose",
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Pose Subscriber Node has been started.')

    def pose_callback(self, msg):
        self.get_logger().info(
            f'Received Pose - Position: (x: {msg.position.x}, y: {msg.position.y}, z: {msg.position.z}), '
            f'Orientation: (x: {msg.orientation.x}, y: {msg.orientation.y}, z: {msg.orientation.z}, w: {msg.orientation.w})'
        )

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber_node = PoseSubscriberNode()
    rclpy.spin(pose_subscriber_node)
    rclpy.shutdown()    