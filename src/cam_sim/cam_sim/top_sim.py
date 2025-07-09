import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib
import random
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

        self.points = self.generate_points()
        self.transformed_points = []
        self.timer = self.create_timer(0.02, self.update)

    def generate_points(self):
        points = []
        point_count = 5

        for i in range(point_count):
            x = -self.bound_width
            if i%2 == 0: y = random.uniform(-self.depth/2, 0)
            else: y = random.uniform(0, self.depth/2)
            z = random.uniform(-self.height/4, 0)
            points.append({'num': i, 'x': x, 'y': y, 'z': z})

        for i in range(point_count):
            x = self.bound_width
            if i%2 == 0: y = random.uniform(0, self.depth/2)
            else: y = random.uniform(-self.depth/2, 0)
            z = random.uniform(-self.height/4, 0)
            points.append({'num': i + point_count, 'x': x, 'y': y, 'z': z})

        return points
    
    def plot(self):
        for side in [-1, 1]:
            x = side * self.bound_width
            plt.plot([x, x], [-self.height/2, self.height/2], color = 'black', linestyle = '-')

        if abs(self.drone_x) < 1 and abs(self.drone_y) < 1: clr = 'green'
        else: clr = 'red'

        plt.plot([0, 0], [-self.height/2, self.height/2], color = clr, linestyle = '--')

        for point in self.points:
            plt.plot(point['x'], point['z'], 'bo', markersize=12)
            plt.plot(point['x'], point['z'], marker = f'${point["num"]}$', markersize = 7, color = 'white')

        plt.plot(self.drone_x, self.drone_z, 'ro', markersize=12)

    def control(self):
        left_vx_sum, right_vx_sum = 0.0, 0.0
        left_vx_avg, right_vx_avg = 0.0, 0.0

        bottom_vy_sum, top_vy_sum = 0.0, 0.0
        bottom_vy_avg, top_vy_avg = 0.0, 0.0

        total_weight = 0.0
        weights = []

        for point in self.points:
            dx = point['x'] - self.drone_x
            dy = point['y'] - self.drone_y
            dz = point['z'] - self.drone_z

            if dz <= 0:
                continue

            Vx = (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)
            Vy = (-self.drone_dy * dz + self.drone_dz * dy) / (dz ** 2)

            speed = (Vx**2 + Vy**2)**0.5
            weight = 1.0 / max(speed, 0.001)

            weights.append((dx, dy, weight, Vx, Vy))
            total_weight += weight

        if total_weight == 0:
            total_weight = 1.0

        for dx, dy, weight, Vx, Vy in weights:
            if dx < 0: left_vx_avg += weight * Vx
            else: right_vx_avg += weight * Vx

            if dy < 0: bottom_vy_avg += weight*Vy
            else: top_vy_avg += weight*Vy

        left_vx_avg /= total_weight
        right_vx_avg /= total_weight
        bottom_vy_avg /= total_weight
        top_vy_avg /= total_weight

        for dx, dy, weight, Vx, Vy in weights:
            norm_weight = weight/total_weight

            if dx < 0: left_vx_sum += norm_weight * (Vx - left_vx_avg)
            else: right_vx_sum += norm_weight * (Vx - right_vx_avg)

            if dy < 0: bottom_vy_sum += norm_weight * (Vy - bottom_vy_avg)
            else: top_vy_sum += norm_weight * (Vy - top_vy_avg)

        plt.plot(20, self.drone_z, 'ko', markersize=8)
        plt.plot(-20, self.drone_z, 'ko', markersize=8)
        plt.quiver(20, self.drone_z, 8000 * right_vx_sum, 0, angles='xy', scale_units='xy', scale=1, color='red')
        plt.quiver(-20, self.drone_z, 8000 * left_vx_sum, 0, angles='xy', scale_units='xy', scale=1, color='red')

        self.drone_dx = left_vx_sum + right_vx_sum
        self.drone_dy = bottom_vy_sum + top_vy_sum
        self.drone_dz = 1.0

        self.drone_x += self.drone_dx
        self.drone_y += self.drone_dy
        self.drone_z += self.drone_dz
        self.frame_count += 1

    def transform(self):
        self.transformed_points = []

        for point in self.points:
            dx = point['x'] - self.drone_x
            dy = point['y'] - self.drone_y
            dz = point['z'] - self.drone_z

            if dz <= 0:
                continue

            x_proj = self.focal_length * (dx / dz)
            y_proj = self.focal_length * (dy / dz)

            Vx = (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)
            Vy = (-self.drone_dy * dz + self.drone_dz * dy) / (dz ** 2)

            self.transformed_points.append({'num': point['num'], 
                'x': x_proj, 'y': y_proj, 
                'world_x': point['x'], 'world_y': point['y'], 'world_z': point['z'], 
                'Vx': Vx, 'Vy': Vy})

    def update(self):
        plt.clf()
        
        if self.drone_z == self.height/2: 
            return
        
        if (self.frame_count == 120): self.drone_x = 10

        for point in self.points:
            if point['z'] < self.drone_z + self.height/4: 
                point['z'] = random.uniform(self.drone_z + self.height/4, self.drone_z + self.height/2)
                if point['y'] < 0: point['y'] = random.uniform(-self.depth/2, 0)
                else: point['y'] = random.uniform(0, self.depth/2)

        self.control()
        self.transform()

        os.system("clear")
        print(f"Drone Position: x = {self.drone_x:.2f}, y = {self.drone_y:.2f}, z = {self.drone_z:.2f}\n")

        for i, point in enumerate(self.transformed_points):
            print(f"Point {point['num']}: x = {point['world_x']:.2f}, y = {point['world_y']:.2f}, z = {point['world_z']:.2f}")
            print(f"Drone Frame: x = {point['x']:.2f}, y = {point['y']:.2f}")
            print(f"Velocity: Vx = {point['Vx']*1000:.2f}, Vy = {point['Vy']*1000:.2f}\n")
        
        self.plot()

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