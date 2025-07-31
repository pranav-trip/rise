import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import time, sys, termios, tty, select

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.cmd_pub = self.create_publisher(String, 'drone_command', 10)
        self.img_pub = self.create_publisher(Bool, 'drone_stream', 10)

        time.sleep(10)
        self.timer = self.create_timer(0.1, self.send_command)
        self.time_lim = 230
        self.time_count = 0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                return sys.stdin.read(3)
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def get_command(self):
        key = self.get_key()

        if key == '\x1b[A': data = 'forward'
        elif key == '\x1b[B': data = 'backward'
        elif key == '\x1b[C': data = 'right'
        elif key == '\x1b[D': data = 'left'
        else: data = 'none'

        return data

    def send_command(self):
        if self.time_count < self.time_lim:
            msg = String()
            msg.data = 'forward'
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Command: {msg.data}")
            
            msg = Bool()
            msg.data = True
            self.img_pub.publish(msg)
            
            self.time_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()