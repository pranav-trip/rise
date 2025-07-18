import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.cmd_pub = self.create_publisher(Twist, 'drone_command', 10)
        self.img_pub = self.create_publisher(Bool, 'drone_stream', 10)

        self.timer = self.create_timer(0.1, self.send_commands)
        self.time_count = 0
        self.time_lim = 120

    def send_commands(self):
        if self.time_count < self.time_lim:
            msg = Twist()
            msg.linear.x = 25.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Command: {msg.linear.x} m/s")

        else:
            msg = Twist()
            msg.linear.x = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Command: {msg.linear.x} m/s")
            self.get_logger().info("Finished")
            self.timer.destroy()
            return
        
        msg = Bool()
        if (self.time_count+1)%10 == 0: msg.data = True
        else: msg.data = False
        self.img_pub.publish(msg)
        self.get_logger().info("Command: Show Frame")
        
        self.time_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()