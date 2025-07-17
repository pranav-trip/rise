import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from djitellopy import Tello
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscriber = self.create_subscription(Twist, 'drone_command', self.control, 10)

        self.tello = Tello()
        self.tello.connect(wait_for_state=False)
        self.get_logger().info("Tello Connected")

        time.sleep(1)
        self.tello.takeoff()

        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.get_logger().info("Ready for Commands")

    def control(self, msg):
        self.tello.send_rc_control(0, int(msg.linear.x), 0, 0)
        self.last_cmd_time = time.time()
        self.get_logger().info(f"Received command: {msg.linear.x}")

    def check_timeout(self):
        if time.time() - self.last_cmd_time > 5.0:
            self.get_logger().info("Tello Disconnected")
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.land()
            self.timer.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()