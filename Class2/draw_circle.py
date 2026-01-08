from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('draw_circle_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.send_velocity_command)
        self.get_logger().info('Draw Circle Node has been started.')

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 0.2  # Forward speed
        msg.angular.z = 0.5  # Angular speed for circular motion
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    draw_circle_node = DrawCircleNode()
    rclpy.spin(draw_circle_node)
    rclpy.shutdown()