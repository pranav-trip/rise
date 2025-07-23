import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from djitellopy import Tello
from dt_apriltags import Detector

import numpy as np
import cv2, os, time, logging
from collections import deque

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        '''self.cmd_sub = self.create_subscription(Twist, 'drone_command', self.control, 10)
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

        self.points, self.old_points = None, None
        self.ids, self.old_ids = None, None'''

        self.stored_frames = deque(maxlen = 9)
        self.dt = 0.1
        self.kernel = np.array([-4, -3, -2, -1, 0, 1, 2, 3, 4], dtype=np.float32)
        self.kernel = self.kernel * (1 / np.sum(np.square(np.arange(-4, 5))))

        self.test_velocity_estimation(self.kernel, 1.0)

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

        mask_left[height // 4 : 3 * height // 4, width // 7 : 3 * width // 7] = 255
        mask_right[height // 4 : 3 * height // 4, 4 * width // 7 : 6 * width // 7] = 255

        detections = self.tag_detector.detect(image_bw)
        centers, ids = [], []
        
        for detection in detections:
            x, y = int(detection.center[0]), int(detection.center[1])

            if (0 < y < mask_left.shape[0] and 0 < x < mask_left.shape[1] and (mask_left[y, x] == 255 or mask_right[y, x] == 255)):
                centers.append([x, y])
                ids.append(detection.tag_id)
                cv2.circle(image, (x, y), 5, (255, 0, 0), -1)
            
        self.points = np.array(centers, dtype=np.float32).reshape(-1, 1, 2)
        self.ids = ids

        current_frame = {id: tuple(center) for id, center in zip(ids, centers)}

        if self.image_count > 0 and self.image_count%10 == 0:
            self.compute_vels()
        else:
            self.stored_frames.append(current_frame)

        if self.image_count >= 0:
            overlay = cv2.cvtColor(mask_left, cv2.COLOR_GRAY2BGR)
            overlay[np.where(mask_left==255)] = (0, 255, 0)
            overlay[np.where(mask_right==255)] = (0, 255, 0)
            blended = cv2.addWeighted(image.copy(), 0.9, overlay, 0.1, 0)
            image_path = os.path.join(self.save_dir, f"frame_{self.image_count:04d}.png")
            cv2.imwrite(image_path, blended)

        self.old_ids = self.ids.copy()
        self.old_points = self.points.copy()

        self.image_count += 1

    def compute_vels(self):
        valid_ids = set.intersection(*[set(frame.keys()) for frame in self.stored_frames])

        #os.system("clear")
        print("\n\n")
        for id in sorted(valid_ids):
            x = np.array([frame[id][0] for frame in self.stored_frames], dtype=np.float32)
            y = np.array([frame[id][1] for frame in self.stored_frames], dtype=np.float32)

            vx = float(np.dot(self.kernel, x) / self.dt)
            vy = float(np.dot(self.kernel, y) / self.dt)

            print(f"Tag {id:>2}: vx = {vx:+.3f}, vy = {vy:+.3f}")

    def timeout(self):
        if time.time() - self.last_command > 5.0 and self.command_recieved == True:
            self.get_logger().info("Tello Disconnected")
            self.tello.send_rc_control(0, 0, 0, 0)

            self.tello.land()
            self.tello.streamoff()
            self.get_logger().info(f"Battery Status: {self.tello.get_battery()}")

            cv2.destroyAllWindows()
            self.timer.destroy()

    def test_velocity_estimation(self, kernel, dt):
        def compute_velocity_from_track(track_x, track_y):
            vx = float(np.dot(kernel, track_x) / dt)
            vy = float(np.dot(kernel, track_y) / dt)
            return vx, vy

        print("=== TEST 1: Perfect Linear Motion (vx=2.0, vy=1.0) ===")
        track_x = [i * 2 for i in range(9)]
        track_y = [i * 1 for i in range(9)]
        vx, vy = compute_velocity_from_track(track_x, track_y)
        print(f"vx = {vx:.3f}, vy = {vy:.3f}")

        print("\n=== TEST 2: Noisy Linear Motion (vx≈2.0, vy≈1.0) ===")
        np.random.seed(0)
        track_x_noisy = [i * 2 + np.random.normal(0, 0.1) for i in range(9)]
        track_y_noisy = [i * 1 + np.random.normal(0, 0.1) for i in range(9)]
        vx, vy = compute_velocity_from_track(track_x_noisy, track_y_noisy)
        print(f"vx = {vx:.3f}, vy = {vy:.3f}")

        print("\n=== TEST 3: Zero Velocity Test (vx=0.0, vy=0.0) ===")
        track_x_zero = [5] * 9
        track_y_zero = [10] * 9
        vx, vy = compute_velocity_from_track(track_x_zero, track_y_zero)
        print(f"vx = {vx:.3f}, vy = {vy:.3f}")

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