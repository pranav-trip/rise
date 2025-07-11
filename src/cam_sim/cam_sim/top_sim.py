import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib
import random
import numpy as np
import os

matplotlib.use('Qt5Agg')

class topSim(Node):
    def __init__(self):
        super().__init__('top_sim')

        self.height = 200
        self.width = 200
        self.depth = 100
        self.bound_width = 60

        self.focal_length = 60

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, -self.height/2
        self.drone_dx, self.drone_dy, self.drone_dz = 0.0, 0.0, 1.0

        self.frame_count = 0
        self.point_count = 5

        self.points = self.generate_points()
        self.transformed_points = []
        self.timer = self.create_timer(0.02, self.update)

        self.plotted, self.vels = [], []

    def generate_points(self):
        points = []

        for i in range(self.point_count):
            x = -self.bound_width
            if i%2 == 0: y = random.uniform(-self.depth/2, 0)
            else: y = random.uniform(0, self.depth/2)
            z = random.uniform(-self.height/4, 0)
            points.append({'num': i, 'x': x, 'y': y, 'z': z})

        for i in range(self.point_count):
            x = self.bound_width
            if i%2 == 0: y = random.uniform(0, self.depth/2)
            else: y = random.uniform(-self.depth/2, 0)
            z = random.uniform(-self.height/4, 0)
            points.append({'num': i + self.point_count, 'x': x, 'y': y, 'z': z})

        return points

    def plot(self):
        for side in [-1, 1]:
            x = side * self.bound_width
            plt.plot([x, x], [-self.height/2, self.height/2], color = 'black', linestyle = '-')

        if abs(self.drone_x) < self.bound_width/12 and abs(self.drone_y) < self.bound_width/12: clr = 'green'
        else: clr = 'red'

        for side in [-1, 1]:
            x = side * self.bound_width/12
            plt.plot([x, x], [-self.height/2, self.height/2], color = clr, linestyle = '--')

        for point in self.points:
            plt.plot(point['x'], point['z'], 'bo', markersize=12)
            plt.plot(point['x'], point['z'], marker = f'${point["num"]}$', markersize = 7, color = 'white')
    
    def weight_vels(self, drone_x, drone_y, drone_z):
        left_vx_avg, right_vx_avg = 0.0, 0.0
        bottom_vy_avg, top_vy_avg = 0.0, 0.0

        vels = []
        max_vx, min_vx = -float('inf'), float('inf')
        max_vy, min_vy = -float('inf'), float('inf')
        total_weight = 0.0

        for point in self.points:
            dx = point['x'] - drone_x
            dy = point['y'] - self.drone_y
            dz = point['z'] - self.drone_z

            if dz <= 0:
                continue

            Vx = (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)
            Vy = (-self.drone_dy * dz + self.drone_dz * dy) / (dz ** 2)

            if (abs(Vx) > max_vx): max_vx = abs(Vx)
            elif (abs(Vx) < min_vx): min_vx = abs(Vx)
            if (abs(Vy) > max_vy): max_vy = abs(Vy)
            elif (abs(Vy) < min_vy): min_vy = abs(Vy)

            vels.append({'dx': dx, 'dy': dy, 'Vx': Vx, 'Vy': Vy})
        
        for vel in vels:
            vel['Vx'] = (abs(vel['Vx']) - min_vx) / (max_vx - min_vx)
            vel['Vy'] = (abs(vel['Vy']) - min_vy) / (max_vy - min_vy)

            if vel['dx'] < 0: vel['Vx'] *= -1
            if vel['dy'] < 0: vel['Vy'] *= -1

        if total_weight == 0:
            total_weight = 1.0

        for vel in vels:
            if vel['dx'] < 0: left_vx_avg += vel['Vx']
            else: right_vx_avg += vel['Vx']

            if vel['dy'] < 0: bottom_vy_avg += vel['Vy']
            else: top_vy_avg += vel['Vy']
        
        scale = 0.25

        left_vx_avg = (left_vx_avg / self.point_count) * scale
        right_vx_avg = (right_vx_avg / self.point_count) * scale
        bottom_vy_avg = (bottom_vy_avg / self.point_count) * scale
        top_vy_avg = (top_vy_avg / self.point_count) * scale

        drone_dx = (left_vx_avg + right_vx_avg)
        drone_dy = (bottom_vy_avg + top_vy_avg)
        drone_dz = 1.0

        return drone_dx, drone_dy, drone_dz, left_vx_avg, right_vx_avg

    def static_vels(self, drone_x, drone_z, point_x, point_z):
        dx = point_x - drone_x
        dz = point_z - drone_z
        return  -1 * (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)

    def control(self):
        self.drone_dx, self.drone_dy, self.drone_dz, left_vx_avg, right_vx_avg = self.weight_vels(self.drone_x, self.drone_y, self.drone_z)

        self.drone_x += self.drone_dx
        self.drone_y += self.drone_dy
        self.drone_z += self.drone_dz
        self.frame_count += 1

        plt.plot(self.drone_x, self.drone_z, 'ro', markersize=12)
        plt.plot(80, 0, 'ko', markersize=10)
        plt.plot(-80, 0, 'ko', markersize=10)
        plt.quiver(80, 0, 200*right_vx_avg, 0, angles='xy', scale_units='xy', scale=1, color='red')
        plt.quiver(-80, 0, 200*left_vx_avg, 0, angles='xy', scale_units='xy', scale=1, color='red')

    def control_field(self):
        field_x = [-50, -35, -20, 20, 35, 50]
        field_z = [-75, -50, -25, 0, 25, 50, 75]

        for x in field_x:
            for z in field_z:
                if z < self.drone_z + self.height / 8:
                    if (x, z) not in self.plotted:
                        dx, dy, dz, left_vel, right_vel = self.weight_vels(x, 0, z)
                        plt.plot(x, z, 'bo', markersize=6)
                        plt.quiver(x, z, 12 * dx, 12 * dz, angles='xy', scale_units='xy', scale=1, color='red')
                        self.plotted.append((x, z))
                        self.vels.append((dx, dz))
                    
                    else:
                        index = self.plotted.index((x, z))
                        dx, dz = self.vels[index]
                        plt.plot(x, z, 'bo', markersize=6)
                        plt.quiver(x, z, 12 * dx, 12 * dz, angles='xy', scale_units='xy', scale=1, color='red')

    def test_control(self):
        test_points = [
            (-40, -60),
            (-40, 60),
            (40, 0),
        ]

        wall_points = [
            (-60, -50),
            (-60, 70),
            (60, 10),
        ]

        for i in range (3):
            drone_x, drone_z = test_points[i]
            x, z = wall_points[i]
            dx = self.static_vels(drone_x, drone_z, x, z)
            
            plt.plot(drone_x, drone_z, 'bo', markersize=8)
            plt.plot(x, z, 'go', markersize=8)
            plt.quiver(drone_x, drone_z, 15 * dx, 15, angles='xy', scale_units='xy', scale=1, color='red')
            
    def transform(self):
        self.transformed_points = []

        for point in self.points:
            dx = point['x'] - self.drone_x
            dy = point['y'] - self.drone_y
            dz = point['z'] - self.drone_z

            if dz <= 0:
                continue

            Vx = (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)
            Vy = (-self.drone_dy * dz + self.drone_dz * dy) / (dz ** 2)

            self.transformed_points.append({'num': point['num'], 'x': point['x'], 'y': point['y'], 'z': point['z'], 'Vx': Vx, 'Vy': Vy})

    def update(self):
        plt.clf()
        
        if self.drone_z == self.height/2: 
            return

        for point in self.points:
            if point['z'] < self.drone_z + self.height/4: 
                point['z'] = random.uniform(self.drone_z + self.height/4, self.drone_z + self.height/2)
                if point['y'] < 0: point['y'] = random.uniform(-self.depth/2, 0)
                else: point['y'] = random.uniform(0, self.depth/2)

        self.plot()
        self.control()
        self.control_field()
        self.transform()

        os.system("clear")
        print(f"Drone Position: x = {self.drone_x:.2f}, y = {self.drone_y:.2f}, z = {self.drone_z:.2f}\n")

        for i, point in enumerate(self.transformed_points):
            print(f"Point {point['num']}: x = {point['x']:.2f}, y = {point['y']:.2f}, z = {point['z']:.2f}")
            print(f"Velocity: Vx = {point['Vx']*1000:.2f}, Vy = {point['Vy']*1000:.2f}\n")

        plt.axis('equal')
        plt.xlim(-self.width/2, self.width / 2)
        plt.ylim(-self.height/2 , self.height / 2)
        plt.axis('off')
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = topSim() 

    plt.ion()
    plt.show(block=False)
    fig = plt.gcf()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()