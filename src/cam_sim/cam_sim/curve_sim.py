import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib
import random
import numpy as np
import os

matplotlib.use('Qt5Agg')

class curveSim(Node):
    def __init__(self):
        super().__init__('curve_sim')

        self.bound_width = 60
        self.window = np.pi/12

        self.radius = 100
        self.angle_range = np.linspace(0, np.pi, 500)

        self.drone_theta = 0.0
        self.drone_x = self.radius * np.cos(self.drone_theta)
        self.drone_z = self.radius * np.sin(self.drone_theta)

        self.drone_dx, self.drone_dz, self.drone_dt = 0.0, 0.0, 0.0

        self.timer = self.create_timer(0.01, self.update)

        self.point_count = 4
        self.points = self.generate_points() 

    def generate_points(self):
        points = []

        for i in range(self.point_count):
            left_radius = self.radius - self.bound_width
            right_radius = self.radius + self.bound_width

            left_angle = random.uniform(self.drone_theta + self.window*2, self.drone_theta + self.window*3)
            right_angle = random.uniform(self.drone_theta + self.window*2, self.drone_theta + self.window*3)

            x_left = left_radius * np.cos(left_angle)
            z_left = left_radius * np.sin(left_angle)

            x_right = right_radius * np.cos(right_angle)
            z_right = right_radius * np.sin(right_angle)

            points.append({'num': i, 'x': x_left, 'z': z_left, 'theta': left_angle, 'wall': 'left'})
            points.append({'num': i + self.point_count, 'x': x_right, 'z': z_right, 'theta': right_angle, 'wall': 'right'})

        return points

    def plot(self):
        left_wall = self.radius - self.bound_width
        right_wall = self.radius + self.bound_width

        left_wall_x = left_wall * np.cos(self.angle_range)
        left_wall_y = left_wall * np.sin(self.angle_range)
        right_wall_x = right_wall * np.cos(self.angle_range)
        right_wall_y = right_wall * np.sin(self.angle_range)

        plt.plot(left_wall_x, left_wall_y, color='black', linestyle='-')
        plt.plot(right_wall_x, right_wall_y, color='black', linestyle='-')

        left_center = self.radius - self.bound_width/10
        right_center = self.radius + self.bound_width/10

        left_center_x = left_center * np.cos(self.angle_range)
        left_center_z = left_center * np.sin(self.angle_range)
        right_center_x = right_center * np.cos(self.angle_range)
        right_center_z = right_center * np.sin(self.angle_range)

        drone_radius = np.sqrt(self.drone_x**2 + self.drone_z**2)
        
        if self.radius-5 <= drone_radius <= self.radius+5: clr = 'green'
        else: clr = 'red'

        plt.plot(left_center_x, left_center_z, color=clr, linestyle='--')
        plt.plot(right_center_x, right_center_z, color=clr, linestyle='--')

        for point in self.points:
            plt.plot(point['x'], point['z'], 'bo', markersize=10)
            plt.plot(point['x'], point['z'], marker=f'${point["num"]}$', markersize=6, color='white')

    def weight_vels(self, drone_x, drone_z):
        left_vx_avg, right_vx_avg = 0.0, 0.0

        vels = []
        max_vx, min_vx = -float('inf'), float('inf')
        total_weight = 0.0

        for point in self.points:
            dx = point['x'] - drone_x
            dz = point['z'] - drone_z

            Vx = (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)

            if (abs(Vx) > max_vx): max_vx = abs(Vx)
            elif (abs(Vx) < min_vx): min_vx = abs(Vx)

            vels.append({'dx': dx, 'Vx': Vx, 'wall': point['wall']})
        
        for vel in vels:
            vel['Vx'] = (abs(vel['Vx']) - min_vx) / max((max_vx - min_vx), 0.001)

            if vel['wall'] == 'left': vel['Vx'] *= -1

        if total_weight == 0:
            total_weight = 1.0

        for vel in vels:
            if vel['dx'] < 0: left_vx_avg += vel['Vx']
            else: right_vx_avg += vel['Vx']
        
        scale = 0.25

        left_vx_avg = (left_vx_avg / self.point_count) * scale
        right_vx_avg = (right_vx_avg / self.point_count) * scale

        drone_dx = (left_vx_avg + right_vx_avg)
        drone_dz = 1.0

        return drone_dx, drone_dz

    def control(self):
        dx, dz = self.weight_vels(self.drone_x, self.drone_z)

        self.drone_dt = 0.01
        self.drone_theta += self.drone_dt

        self.drone_dx = dx * np.cos(self.drone_theta) - dz * np.sin(self.drone_theta)
        self.drone_dz = dx * np.sin(self.drone_theta) + dz * np.cos(self.drone_theta)

        self.drone_x += self.drone_dx
        self.drone_z += self.drone_dz

        plt.plot(self.drone_x, self.drone_z, 'ro', markersize=10)
        plt.quiver(self.drone_x, self.drone_z, 20 * self.drone_dx, 20 * self.drone_dz, angles='xy', scale_units='xy', scale=1, color='blue')

    def update(self):
        plt.clf()
        end_sim = True

        for point in self.points:
            if point['theta'] < np.pi: end_sim = False
        
        if end_sim: return

        for point in self.points:
            theta = point['theta']

            if theta < self.drone_theta + self.window*2:
                theta = random.uniform(self.drone_theta + self.window*2, self.drone_theta + self.window*3)
                if theta > np.pi: theta = np.pi 

                if point['wall'] == 'left': radius = self.radius - self.bound_width
                else: radius = self.radius + self.bound_width

                point['x'] = radius * np.cos(theta)
                point['z'] = radius * np.sin(theta)
                point['theta'] = theta

        self.plot()
        self.control()

        plt.axis('equal')

        arc_lim = self.radius + self.bound_width
        plt.xlim(-arc_lim, arc_lim)
        plt.ylim(0, arc_lim)

        plt.axis('off')
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = curveSim() 

    plt.ion()
    plt.show(block=False)
    fig = plt.gcf()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()