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

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.cmd_sub = self.create_subscription(Twist, 'drone_command', self.control, 10)
        self.img_sub = self.create_subscription(Bool, 'drone_stream', self.stream, 10)
        logging.getLogger("djitellopy").setLevel(logging.ERROR)

        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info("Tello Connected")
        self.command_recieved = False

<<<<<<< HEAD
        self.image_count = -24
=======
        self.image_count = -22
>>>>>>> @{-1}
        self.save_dir = os.path.expanduser('~/rise/images')

        time.sleep(1)
        self.tello.takeoff()
        self.tello.streamon()

        self.last_command = time.time()
        self.timer = self.create_timer(0.1, self.timeout)
        self.get_logger().info("Ready for Commands")

<<<<<<< HEAD
        self.bw, self.points, self.old_points = None, None, None
=======
        self.points, self.old_points, self.bw = None, None, None
        self.calc_vel = True
>>>>>>> @{-1}

    def control(self, msg):
        self.tello.send_rc_control(0, int(msg.linear.x), 0, 0)
        self.last_command = time.time()
        self.command_recieved = True

    def stream(self, msg):
        if not msg.data:   
            self.get_logger().info("failed")
            return

        image = self.tello.get_frame_read().frame
        image_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

<<<<<<< HEAD
        height, width = image_bw.shape
        mask = np.zeros_like(image_bw, dtype=np.uint8)
        mask[:, width // 5 : 2 * width // 5] = 255
        mask[:, 3 * width // 5 : 4 * width // 5] = 255

        if self.bw is None or self.points is None or len(self.points) < 10:
            self.bw = image_bw.copy()
            self.points = cv2.goodFeaturesToTrack(image_bw, mask=mask, maxCorners=10, qualityLevel=0.7, minDistance=0.5)
            self.image_count +=1
            return
        
        new_points, status, _ = cv2.calcOpticalFlowPyrLK(self.bw, image_bw, self.points, None)
        tracked_points = new_points[status == 1]
=======
        new_points = None
        if self.points is not None:
            new_points, status, _ = cv2.calcOpticalFlowPyrLK(self.bw, image_bw, self.points, None)
            new_points = new_points[status.flatten() == 1]
>>>>>>> @{-1}

        if new_points is None or len(new_points) < 10:
            self.points = cv2.goodFeaturesToTrack(image_bw, maxCorners=10, qualityLevel=0.5, minDistance=0.5)
            self.calc_vel = False
            self.old_points = None

        else:
            self.points = new_points
            self.calc_vel = True

        if self.points is not None:
            for point in self.points:
                x, y = point.ravel()
                cv2.circle(image, (int(x), int(y)), 5, (255, 0, 0), -1)

        if self.image_count >= 0:
            image_path = os.path.join(self.save_dir, f"frame_{self.image_count:04d}.png")
            cv2.imwrite(image_path, image)

            if self.calc_vel and self.old_points is not None:
                os.system("clear")
                for i, point in enumerate(self.points):
                    x, y = point.ravel()
                    xo, yo = self.old_points[i].ravel()
                    print(f"Point {i}: {(x-xo)*100:.2f} pixels per second")

            if self.calc_vel:
                self.old_points = self.points.copy()

        self.bw = image_bw.copy()
        self.image_count += 1
<<<<<<< HEAD
        
        if not msg.data:
            return

        image_path = os.path.join(self.save_dir, f"frame_{self.image_count:04d}.png")
        if self.image_count >= 0:
            mask_overlay = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_overlay[np.where(mask == 255)] = (0, 255, 0)
            blended = cv2.addWeighted(image.copy(), 0.7, mask_overlay, 0.3, 0)
            cv2.imwrite(image_path, blended)

        if self.old_points is None:
            self.old_points = self.points
        else:
            os.system("clear")
            for i, point in enumerate(self.points):
                if i > len(self.old_points): break

                x, y = point.ravel()               
                xo, yo = self.old_points[i].ravel()

                vel = (x - xo, y - yo)
                print(f"Point {i}: Vx = {vel[0]:.2f}, Vy = {vel[1]:.2f}")

            self.old_points = self.points.copy()
=======
>>>>>>> @{-1}

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