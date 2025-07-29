import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

from djitellopy import Tello
from dt_apriltags import Detector

import numpy as np
import matplotlib.pyplot as plt
import cv2, os, time, logging, random
from collections import deque

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.cmd_sub = self.create_subscription(String, 'drone_command', self.control, 10)
        self.img_sub = self.create_subscription(Bool, 'drone_stream', self.stream, 10)
        logging.getLogger("djitellopy").setLevel(logging.ERROR)

        self.tag_detector = Detector(families='tag36h11')

        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info("Tello Connected")
        self.command_recieved = False

        self.image_count = -18
        self.save_dir = os.path.expanduser('~/rise/images')

        time.sleep(1)
        self.tello.takeoff()
        self.tello.streamon()

        self.last_command = time.time()
        self.timer = self.create_timer(0.1, self.timeout)
        self.get_logger().info("Ready for Commands")

        self.points, self.old_points = None, None
        self.ids, self.old_ids = None, None

        self.stored_frames = deque(maxlen = 9)
        self.kernel = np.array([-4, -3, -2, -1, 0, 1, 2, 3, 4], dtype=np.float32)

        self.signal = 0.0

    def timeout(self):
        if time.time() - self.last_command > 5.0 and self.command_recieved == True:
            self.get_logger().info("Tello Disconnected")
            self.tello.send_rc_control(0, 0, 0, 0)

            self.tello.land()
            self.tello.streamoff()
            self.get_logger().info(f"Battery Status: {self.tello.get_battery()}")

            cv2.destroyAllWindows()
            self.timer.destroy()

    def control(self, msg):
        command = msg.data

        if command == 'forward': self.tello.send_rc_control(4, 15, 0, int(self.signal))
        elif command == 'backward': self.tello.send_rc_control(0, -15, 0, 0)
        elif command == 'right': self.tello.send_rc_control(15, 0, 0, 0)
        elif command == 'left': self.tello.send_rc_control(-15, 0, 0, 0)
        elif command == 'none': self.tello.send_rc_control(0, 0, 0, 0)

        self.last_command = time.time()
        self.command_recieved = True

    def stream(self, msg):
        if not msg.data: return

        image = self.tello.get_frame_read().frame
        image_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        height, width = image_bw.shape
        mask_left = np.zeros_like(image_bw, dtype=np.uint8)
        mask_right = np.zeros_like(image_bw, dtype=np.uint8)

        mask_left[3 * height // 8 : 6 * height // 8, width // 10 : 4 * width // 10] = 255
        mask_right[3 * height // 8 : 6 * height // 8, 6 * width // 10 : 9 * width // 10] = 255

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
            self.compute_vels(mask_left, mask_right)
        else:
            self.stored_frames.append(current_frame)

        if self.image_count >= 0:
            overlay = cv2.cvtColor(mask_left, cv2.COLOR_GRAY2BGR)
            overlay[np.where(mask_left==255)] = (0, 255, 0)
            overlay[np.where(mask_right==255)] = (0, 255, 0)
            blended = cv2.addWeighted(image.copy(), 0.8, overlay, 0.2, 0)
            image_path = os.path.join(self.save_dir, f"frame_{self.image_count:04d}.png")
            cv2.imwrite(image_path, blended)

        self.old_ids = self.ids.copy()
        self.old_points = self.points.copy()

        self.image_count += 1

    def compute_vels(self, mask_left, mask_right):
        valid_ids = set.intersection(*[set(frame.keys()) for frame in self.stored_frames])
        vels = []

        print(f"\n\n{self.image_count:04d}")
        for id in sorted(valid_ids):
            x = np.array([frame[id][0] for frame in self.stored_frames], dtype=np.float32)
            y = np.array([frame[id][1] for frame in self.stored_frames], dtype=np.float32)

            square_sum = np.sum(np.square(self.kernel))

            vx = float(np.dot(self.kernel, x) / square_sum)
            vy = float(np.dot(self.kernel, y) / square_sum)

            if mask_left[int(y[0]), int(x[0])] == 255: side = 'left'
            if mask_right[int(y[0]), int(x[0])] == 255: side = 'right'
            
            vels.append({'vx': vx, 'vy': vy, 'side': side, 'id': id})
        
        self.compute_signal(vels)

    def compute_signal(self, vels):
        min_vx, max_vx = float('inf'), -float('inf')
        left_vx_avg, right_vx_avg = 0, 0
        left_count, right_count = 0, 0
        amp = 2.8
        
        for vel in vels:
            vel['vx'] = abs(vel['vx'])
            if vel['vx'] < min_vx: min_vx = vel['vx']
            if vel['vx'] > max_vx: max_vx = vel['vx']
        
        for vel in vels:
            vel['vx'] = max((abs(vel['vx']) - min_vx) / max((max_vx - min_vx), 0.1), 0.1)
            if vel['side'] == 'left': 
                left_vx_avg += vel['vx']
                left_count += 1
            else:
                right_vx_avg += vel['vx']
                right_count += 1
            
            print(f"Tag {vel['id']:>2}: vx = {vel['vx']:.3f}, side = {vel['side']}")
        
        if left_count > 0: left_vx_avg /= left_count
        else: left_vx_avg = 1.0

        if right_count > 0: right_vx_avg /= right_count
        else: right_vx_avg = 1.0

        signal = right_vx_avg - left_vx_avg
        
        self.left_vx = left_vx_avg
        self.right_vx = right_vx_avg

        if np.sign(self.signal) != np.sign(signal) or self.signal == 0: 
            self.signal = 0    
            amp *= 1.8
        
        self.signal += signal * amp

        print(f"\n\nLeft Vx: {left_vx_avg:.3f}, Right Vx: {right_vx_avg:.3f}, New Signal: {signal:.3f}, Signal: {self.signal:.3f}")

    def test_vels(self, kernel):
        def compute_vel(x, y):
            square_sum = np.sum(np.square(np.arange(-4, 5)))
            vx = float(np.dot(kernel, x) / square_sum)
            vy = float(np.dot(kernel, y) / square_sum)
            return vx, vy

        print("TEST 1: Perfect Linear Motion (vx=10.0, vy=7.0)")
        x = [i * 10 for i in range(9)]
        y = [i * 7 for i in range(9)]
        vx, vy = compute_vel(x, y)
        print(f"vx = {vx:.3f}, vy = {vy:.3f}")

        print("\nTEST 2: Noisy Linear Motion (vx≈10.0, vy≈8.0)")
        np.random.seed(0)
        x_noisy = [i * 5 + random.random()*3 for i in range(9)]
        y_noisy = [i * 8 - random.random()*3 for i in range(9)]

        vxo = []
        for i in range(len(x_noisy)):
            if i == 0:
                v = x_noisy[1] - x_noisy[0]
            elif i == len(x_noisy) - 1:
                v = x_noisy[-1] - x_noisy[-2]
            else:
                v = (x_noisy[i + 1] - x_noisy[i - 1]) / 2
            vxo.append(v)
        
        vx, vy = compute_vel(x_noisy, y_noisy)
        vx_mean = (vxo[0]+vxo[len(vxo)-1]) / 2

        print(f"vx = {vx:.3f}, vy = {vy:.3f}")
        print(f"mean vx = {vx_mean:.3f}")

        plt.figure(figsize=(12, 5))

        plt.plot(range(len(vxo)), vxo, color='red', label='Raw Vx', linestyle='--')
        plt.axhline(vx_mean, color='green', label='Average Vx', linestyle='-')
        plt.axhline(vx, color='blue', label='Filtered Vx (Gaussian)', linestyle='-')

        plt.xlabel("Frame Index")
        plt.ylabel("Velocity (pixels/frame)")
        plt.title("Gaussian Velocity Normalization")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        print("\nTEST 3: Zero Velocity Test (vx=0.0, vy=0.0)")
        x_zero = [5] * 9
        y_zero = [10] * 9
        vx, vy = compute_vel(x_zero, y_zero)
        print(f"vx = {vx:.3f}, vy = {vy:.3f}")

    def test_signal(self):
        np.random.seed(0)
        raw_vels = np.concatenate([
            np.random.normal(4.0, 0.8, 10),
            np.random.normal(9.0, 0.8, 10)
        ])

        min_val = np.min(raw_vels)
        max_val = np.max(raw_vels)
        norm_vels = (raw_vels - min_val) / (max_val - min_val)

        plt.figure(figsize=(10, 5))
        plt.plot(raw_vels, 'ro', label='Raw Velocities')
        plt.plot(norm_vels, 'bo', label='Normalized Velocities')
        plt.xlabel('Point Index')
        plt.ylabel('Velocity (pixels/second)')
        plt.grid(True)
        plt.title('Min-Max Velocity Normalization')
        plt.legend()
        plt.tight_layout()
        plt.show()


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