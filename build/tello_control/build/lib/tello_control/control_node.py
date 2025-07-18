import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from djitellopy import Tello
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscriber = self.create_subscription(Twist, 'drone_command', self.control, 10)
        self.create_subscription(Bool, 'save_image', self.save_image, 10)

        self.tello = Tello()
        self.tello.connect(wait_for_state=False)
        self.get_logger().info("Tello Connected")

        time.sleep(1)
        self.tello.takeoff()

        self.last_command = time.time()
        self.frame_count = 0
        self.tello.streamon()
        
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

    def save_image(self, msg):
        if msg.data:
            frame = self.tello.get_frame_read().frame
            filename = f"/home/ptrip/ros2_ws/images/frame_{self.frame_count:03}.jpg"
            import cv2
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"Image saved: {filename}")
            self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()