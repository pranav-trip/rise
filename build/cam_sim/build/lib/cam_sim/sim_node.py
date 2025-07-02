import rclpy
from rclpy.node import Node
import math
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

import time

plt.ion()

class camSim(Node):
    def __init__(self):
        super().__init__('cam_sim')
    
        self.drone_x = 10.0
        self.drone_y = 20.0

        self.world_points = [
            (5.0, 4.0),
            (17.0, 27.0),
            (12.0, 36.0)
        ]

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        plt.clf()
        self.get_logger().info(f'Drone Position: x = {self.drone_x}, y={self.drone_y}\n')

        for i, (x, y) in enumerate(self.world_points):
            x_new, y_new = self.transform(x, y)

            plt.plot(x_new, y_new, 'bo')
            plt.text(x_new+0.5, y_new+0.5, f'{i+1}', fontsize=10)

            self.get_logger().info(f'Point {i+1}: x = {x}, y = {y}')
            self.get_logger().info(f'Drone Frame: x = {x_new}, y = {y_new}\n')

        plt.xlim(0, 20) 
        plt.ylim(0, 40)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.draw()
        plt.pause(0.1)

    def transform(self, x, y):
        x_new, y_new = x, y
        return x_new, y_new
    
def main(args=None):
    rclpy.init(args=args)
    node=camSim()

    plt.ion()
    plt.show(block=False)
    time.sleep(0.1)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()