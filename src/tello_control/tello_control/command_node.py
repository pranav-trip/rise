import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.cmd_pub = self.create_publisher(Twist, 'drone_command', 10)
        self.vel_pub = self.create_publisher(Bool, 'save_image', 10)

        self.timer = self.create_timer(0.01, self.send_command)
        self.time_count = 0
        self.time_lim = 1000

    def send_command(self):
        if self.time_count < self.time_lim:
            msg = Twist()
            msg.linear.x = 20.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Command: {msg.linear.x} m/s")
            self.time_count += 1

        else:
            msg = Twist()
            msg.linear.x = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Command: {msg.linear.x} m/s")
            self.get_logger().info("Finished")
            self.timer.destroy()
            return
        
        if self.time_count % 200 == 0:
            msg = Bool()
            msg.data = True
            self.vel_pub.publish(msg)
            self.get_logger().info("Command: Log Velocity")

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()