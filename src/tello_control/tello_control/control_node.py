import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from djitellopy import Tello
import time
import socket

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.cmd_sub = self.create_subscription(Twist, 'drone_command', self.control, 10)
        #self.vel_sub = self.create_subscription(Bool, 'drone_vel', self.log_vel, 10)

        self.tello = Tello()
        self.tello.connect(wait_for_state=False)
        self.get_logger().info("Tello Connected")

        time.sleep(1)
        self.tello.takeoff()

        self.last_command = time.time()
        self.timer = self.create_timer(0.01, self.timeout)
        self.get_logger().info("Ready for Commands")

    def control(self, msg):
        self.tello.send_rc_control(0, int(msg.linear.x), 0, 0)
        self.last_command = time.time()
        self.get_logger().info(f"Received command: {msg.linear.x}")

    def timeout(self):
        if time.time() - self.last_command > 5.0:
            self.get_logger().info("Tello Disconnected")
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.land()
            self.timer.destroy()

    def log_vel(self, msg):
        if msg.data:
            vel = self.tello.get_speed_x()
            self.get_logger().info(f"Velocity: {vel}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info("Shutting Down")
        node.tello.land()
        node.tello.streamoff()
        node.tello.end()
        node.destroy_node()
        rclpy.shutdown()