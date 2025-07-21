import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from djitellopy import Tello
import time
import cv2
import os

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.cmd_sub = self.create_subscription(Twist, 'drone_command', self.control, 10)
        self.img_sub = self.create_subscription(Bool, 'drone_stream', self.stream, 10)

        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info("Tello Connected")
        self.command_recieved = False

        self.image_count = -2
        self.save_dir = os.path.expanduser('~/rise/images')

        time.sleep(1)
        self.tello.takeoff()
        self.tello.streamon()

        self.last_command = time.time()
        self.timer = self.create_timer(0.1, self.timeout)
        self.get_logger().info("Ready for Commands")

        self.bw, self.points = None, None

    def control(self, msg):
        self.tello.send_rc_control(0, int(msg.linear.x), 0, 0)
        self.last_command = time.time()
        self.get_logger().info(f"Received command: {msg.linear.x}")
        self.command_recieved = True

    def stream(self, msg):
        image = self.tello.get_frame_read().frame
        image_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.bw is None or self.points is None or len(self.points) == 0:
            self.bw = image_bw.copy()
            self.points = cv2.goodFeaturesToTrack(image_bw, maxCorners=20, qualityLevel=0.5, minDistance=0.5)
            self.image_count +=1
            return
        
        new_points, status, _ = cv2.calcOpticalFlowPyrLK(self.bw, image_bw, self.points, None)
        tracked_points = new_points[status == 1]

        for point in tracked_points:
            x, y = point.ravel()
            cv2.circle(image, (int(x), int(y)), 5, (255, 0, 0), -1)

        self.bw = image_bw.copy()
        self.points = tracked_points.reshape(-1, 1, 2)
        self.image_count += 1
        
        if not msg.data:
            return

        image_path = os.path.join(self.save_dir, f"frame_{self.image_count:04d}.png")
        if self.image_count >= 0: cv2.imwrite(image_path, image)

    def timeout(self):
        if time.time() - self.last_command > 5.0 and self.command_recieved == True:
            self.get_logger().info("Tello Disconnected")
            self.tello.send_rc_control(0, 0, 0, 0)

            self.tello.land()
            self.tello.streamoff()
            self.get_logger().info(f"Battery Status: {self.tello.get_battery()}")

            cv2.destroyAllWindows()
            self.timer.destroy()


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