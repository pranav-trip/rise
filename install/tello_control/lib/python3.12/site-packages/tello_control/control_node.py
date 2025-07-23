import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from djitellopy import Tello
import numpy as np
import time
import cv2
import os
import logging
import dt_apriltags
from dt_apriltags import Detector

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.cmd_sub = self.create_subscription(Twist, 'drone_command', self.control, 10)
        self.img_sub = self.create_subscription(Bool, 'drone_stream', self.stream, 10)
        logging.getLogger("djitellopy").setLevel(logging.ERROR)

        self.tag_detector = Detector(families='tag36h11')

        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info("Tello Connected")
        self.command_recieved = False

        self.image_count = -12
        self.save_dir = os.path.expanduser('~/rise/images')

        time.sleep(1)
        self.tello.takeoff()
        self.tello.streamon()

        self.last_command = time.time()
        self.timer = self.create_timer(0.1, self.timeout)
        self.get_logger().info("Ready for Commands")

        self.points, self.old_points, self.ids, self.old_ids = None, None, None, None

    def control(self, msg):
        self.tello.send_rc_control(0, int(msg.linear.x), 0, 0)
        self.last_command = time.time()
        self.command_recieved = True

    def stream(self, msg):
        if not msg.data: return

        image = self.tello.get_frame_read().frame
        image_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        height, width = image_bw.shape
        mask_left = np.zeros_like(image_bw, dtype=np.uint8)
        mask_right = np.zeros_like(image_bw, dtype=np.uint8)

        mask_left[height // 2 : , width // 7 : 3 * width // 7] = 255
        mask_right[height // 2 : , 4 * width // 7 : 6 * width // 7] = 255

        detections = self.tag_detector.detect(image_bw)
        centers, ids = [], []
        
        for detection in detections:
            x, y = int(detection.center[0]), int(detection.center[1])

            if (0 < y < mask_left.shape[0] and 0 < x < mask_left.shape[1] and (mask_left[y, x] == 255 or mask_right[y, x] == 255)):
                centers.append([x, y])
                ids.append(detection.tag_id)
            
        self.points = np.array(centers, dtype=np.float32).reshape(-1, 1, 2)
        self.ids = ids

        self.calc_vel = True
        if self.old_ids is not None:
            for id in self.ids:
                if id not in self.old_ids:
                    self.calc_vel = False
                    break
        else:
            self.calc_vel = False

        if self.points is not None:
            for point in self.points:
                x, y = point.ravel()
                cv2.circle(image, (int(x), int(y)), 5, (255, 0, 0), -1)

        if self.image_count >= 0:
            overlay = cv2.cvtColor(mask_left, cv2.COLOR_GRAY2BGR)
            overlay[np.where(mask_left==255)] = (0, 255, 0)
            overlay[np.where(mask_right==255)] = (0, 255, 0)
            blended = cv2.addWeighted(image.copy(), 0.9, overlay, 0.1, 0)
            image_path = os.path.join(self.save_dir, f"frame_{self.image_count:04d}.png")
            cv2.imwrite(image_path, blended)

        if self.calc_vel and self.old_points is not None:
            os.system("clear")
            for i, point in enumerate(self.points):
                if i >= len(self.old_points): break
                x, y = point.ravel()
                xo, yo = self.old_points[i].ravel()
                print(f"Point {i+1}: {(x - xo)*10:.2f} pixels per second")

        self.old_ids = self.ids.copy()
        self.old_points = self.points.copy()

        self.image_count += 1

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
