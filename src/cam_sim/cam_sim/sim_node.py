import rclpy
from rclpy.node import Node
import math
import os
import random
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

class camSim(Node):
    def __init__(self):
        super().__init__('cam_sim')
    
        self.x_lim, self.y_lim = 80, 40

        self.drone_x , self.drone_y = self.x_lim/2, self.y_lim/2
        self.drone_dx, self.drone_dy = 0, 0

        self.world_points = [(random.uniform(0, self.x_lim), random.uniform(0, self.y_lim)) for x in range(12)]

        self.timer = self.create_timer(0.005, self.timer_callback)
        self.last_printed = None
    
    def key_control(self, event):
        step = 0.2
        if event.key == 'right': self.drone_dx = min(self.drone_dx + step, self.x_lim/2)
        elif event.key == 'left': self.drone_dx = max(self.drone_dx - step, -self.x_lim/2)
        elif event.key == 'up': self.drone_dy = min(self.drone_dy + step, self.y_lim/2)
        elif event.key == 'down': self.drone_dy = max(self.drone_dy - step, -self.y_lim/2)

    def timer_callback(self):
        plt.clf()

        self.transformed_points = [self.transform(x, y) for x, y in self.world_points]
        
        if self.last_printed != self.transformed_points:
            os.system("clear")
            self.get_logger().info(f'Drone Position: x = {self.drone_x+self.drone_dx:.2f}, y = {self.drone_y+self.drone_dy:.2f}\n')

            for i, (x, y) in enumerate(self.transformed_points):
                x_org, y_org = self.world_points[i]
                self.get_logger().info(f'Point {i+1}: x = {x_org:.2f}, y = {y_org:.2f}')
                self.get_logger().info(f'Drone Frame: x = {x:.2f}, y = {y:.2f}\n')
            
            self.last_printed = self.transformed_points

        for i, (x, y) in enumerate(self.transformed_points):    
            plt.plot(x, y, 'bo', markersize=12)
            plt.plot(x, y, marker=f'${i+1}$', markersize=8, color='white') 

        
        plt.plot(self.drone_x, self.drone_y, 'ko', markersize=10)
        plt.xlim(self.x_lim*0.25, self.x_lim*0.75) 
        plt.ylim(self.y_lim*0.25, self.y_lim*0.75)

        plt.axis("off")
        plt.draw()
        plt.pause(0.005)

    def transform(self, x, y):
        return x-self.drone_dx, y-self.drone_dy
    
def main(args=None):
    rclpy.init(args=args)
    node = camSim()

    plt.ion()
    plt.show(block=False)

    fig = plt.gcf()
    fig.canvas.mpl_connect('key_press_event', node.key_control)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()