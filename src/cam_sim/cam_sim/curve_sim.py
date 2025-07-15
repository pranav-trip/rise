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
        self.angle_range = np.linspace(0, np.pi/2, 500)

        self.drone_theta = 0.0
        self.drone_x = self.radius * np.cos(self.drone_theta)
        self.drone_z = self.radius * np.sin(self.drone_theta)

        self.drone_dx, self.drone_dz, self.drone_dt = 0.0, 0.0, 0.0

        self.timer = self.create_timer(0.01, self.update)

        self.point_count = 4
        self.points = self.generate_points() 

        self.plotted = []
        self.inner_signals, self.outer_signals = [], []

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

        for vel in vels:
            if vel['dx'] < 0: left_vx_avg += vel['Vx']
            else: right_vx_avg += vel['Vx']

        left_vx_avg = (left_vx_avg / self.point_count)
        right_vx_avg = (right_vx_avg / self.point_count)

        signal = (left_vx_avg + right_vx_avg)
        scale = 0.05 / (1 + 3.5 * abs(signal))

        return signal*scale

    def control(self):
        signal = self.weight_vels(self.drone_x, self.drone_z)

        self.drone_dt = -signal * 0.5
        self.drone_theta += self.drone_dt

        self.drone_dx = -np.sin(self.drone_theta)
        self.drone_dz = np.cos(self.drone_theta)

        self.drone_x += self.drone_dx * 0.5
        self.drone_z += self.drone_dz * 0.5

        plt.plot(self.drone_x, self.drone_z, 'ro', markersize=10)
        plt.quiver(self.drone_x, self.drone_z, 10 * self.drone_dx, 10 * self.drone_dz, angles='xy', scale_units='xy', scale=1, color='blue')

    def control_field(self):
        field_theta = [np.pi/12, np.pi/6, np.pi/4, np.pi/3, 5*np.pi/12]
        inner, outer = self.radius - self.bound_width/2, self.radius + self.bound_width/2

        for theta in field_theta:
                x_inner, z_inner = inner * np.cos(theta), inner * np.sin(theta)
                x_outer, z_outer = outer * np.cos(theta), outer * np.sin(theta)
                scale = 1000

                if theta not in self.plotted:
                    inner_signal = self.weight_vels(x_inner, z_inner)
                    outer_signal = self.weight_vels(x_outer, z_outer)
                    self.plotted.append(theta)

                    self.inner_signals.append(inner_signal)
                    self.outer_signals.append(outer_signal)

                    plt.plot(x_inner, z_inner, 'bo', markersize=6)
                    plt.plot(x_outer, z_outer, 'bo', markersize=6)
                    
                    plt.quiver(x_inner, z_inner, inner_signal*scale * np.cos(theta), inner_signal*scale * np.sin(theta), angles='xy', scale_units='xy', scale=1, color='red')
                    plt.quiver(x_outer, z_outer, outer_signal*scale * np.cos(theta), outer_signal*scale * np.sin(theta), angles='xy', scale_units='xy', scale=1, color='red')

                else:
                    index = field_theta.index(theta)
                    inner_signal = self.inner_signals[index]
                    outer_signal = self.outer_signals[index]

                    plt.plot(x_inner, z_inner, 'bo', markersize=6)
                    plt.plot(x_outer, z_outer, 'bo', markersize=6)
                    
                    plt.quiver(x_inner, z_inner, inner_signal*scale * np.cos(theta), inner_signal*scale * np.sin(theta), angles='xy', scale_units='xy', scale=1, color='red')
                    plt.quiver(x_outer, z_outer, outer_signal*scale * np.cos(theta), outer_signal*scale * np.sin(theta), angles='xy', scale_units='xy', scale=1, color='red')

    def test_control(self):
        test_thetas = [np.pi/18, 2*np.pi/18, 3*np.pi/18, 4*np.pi/18, 5*np.pi/18, 6*np.pi/18, 7*np.pi/18, 8*np.pi/18, 9*np.pi/18]
        wall_radius, wall_theta = self.radius - self.bound_width, 5*np.pi/18 #radius-bound,9*np.pi/12 and radius+bound, 7*np.pi/12
        wall_x, wall_z = wall_radius*np.cos(wall_theta + np.pi/18), wall_radius*np.sin(wall_theta + np.pi/18)

        for theta in test_thetas:
            if (theta >= wall_theta): continue

            drone_x_inner = (self.radius-self.bound_width/2) * np.cos(theta)
            drone_z_inner = (self.radius-self.bound_width/2) * np.sin(theta)
            drone_x_outer = (self.radius+self.bound_width/2) * np.cos(theta)
            drone_z_outer = (self.radius+self.bound_width/2) * np.sin(theta)

            dx_inner = wall_x - drone_x_inner
            dz_inner = wall_z - drone_z_inner
            dx_outer = wall_x - drone_x_outer
            dz_outer = wall_z - drone_z_outer

            drone_dx = -np.sin(theta)
            drone_dz =  np.cos(theta)

            inner_dist = dx_inner**2 + dz_inner**2
            outer_dist = dx_outer**2 + dz_outer**2
            vel_inner = -1*(-drone_dx * dz_inner + drone_dz * dx_inner) / inner_dist
            vel_outer = (-drone_dx * dz_outer + drone_dz * dx_outer) / outer_dist
            vel_inner = np.clip(vel_inner, -10, 10)
            vel_outer = np.clip(vel_outer, -10, 10)

            inner_len = 100 * abs(vel_inner)
            outer_len = 100 * abs(vel_outer)
            inner_arrowx = inner_len * np.cos(theta) * np.sign(vel_inner)
            inner_arrowz = inner_len * np.sin(theta) * np.sign(vel_inner)
            outer_arrowx = outer_len * np.cos(theta) * np.sign(vel_outer)
            outer_arrowz = outer_len * np.sin(theta) * np.sign(vel_outer)

            plt.plot(drone_x_inner, drone_z_inner, 'bo', markersize=8)
            plt.plot(drone_x_outer, drone_z_outer, 'bo', markersize=8)
            plt.plot(wall_x, wall_z, 'go', markersize=8)
            plt.quiver(drone_x_inner, drone_z_inner, inner_arrowx, inner_arrowz, angles='xy', scale_units='xy', scale=1, color='red')
            plt.quiver(drone_x_outer, drone_z_outer, outer_arrowx, outer_arrowz, angles='xy', scale_units='xy', scale=1, color='red')

    def update(self):
        plt.clf()
        end_sim = True

        for point in self.points:
            if point['theta'] < np.pi/2: end_sim = False
        
        if end_sim: return

        for point in self.points:
            theta = point['theta']

            if theta < self.drone_theta + self.window*2:
                theta = random.uniform(self.drone_theta + self.window*2, self.drone_theta + self.window*3)
                if theta > np.pi/2: theta = np.pi/2 

                if point['wall'] == 'left': radius = self.radius - self.bound_width
                else: radius = self.radius + self.bound_width

                point['x'] = radius * np.cos(theta)
                point['z'] = radius * np.sin(theta)
                point['theta'] = theta

        self.plot()
        self.control()

        plt.axis('equal')

        arc_lim = self.radius + self.bound_width
        plt.xlim(0, arc_lim)
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