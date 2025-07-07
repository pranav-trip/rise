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
        self.bound_width = 60

        self.focal_length = 60

        self.drone_x, self.drone_y, self.drone_z = 0.0, 0.0, -self.height/2
        self.drone_dx, self.drone_dy, self.drone_dz = 0.0, 0.0, 0.0

        self.init_frames = 50
        self.frame_count = 0

        self.points = self.generate_points()
        self.timer = self.create_timer(0.02, self.update)

    def generate_points(self):
        points = []

        for i in range(5):
            x = -self.bound_width
            y = random.uniform(-self.height/2, self.height/2)
            z = random.uniform(-self.height/2, self.height/2)
            points.append({'num': i, 'x': x, 'y': y, 'z': z})

        for i in range(5):
            x = self.bound_width
            y = random.uniform(-self.height/2, self.height/2)
            z = random.uniform(-self.height/2, self.height/2)
            points.append({'num': i + 5, 'x': x, 'y': y, 'z': z})

        return points

    def plot(self):
        for side in [-1, 1]:
            x = side * self.bound_width
            plt.plot([x, x], [-self.height/2, self.height/2], color = 'black', linestyle = '-')

        if abs(self.drone_x) < 0.01 and abs(self.drone_y) < 0.01: clr = 'green'
        else: clr = 'red'

        plt.plot([0, 0], [-self.height/2, self.height/2], color = clr, linestyle = '--')

        for point in self.points:
            plt.plot(point['x'], point['z'], 'bo', markersize=12)
            plt.plot(point['x'], point['z'], marker = f'${point["num"]}$', markersize = 7, color = 'white')

        plt.plot(self.drone_x, self.drone_z, 'ro', markersize=12)

    def update(self):
        plt.clf()

        if self.frame_count < self.init_frames:
            self.drone_dz = 1.0
            self.frame_count += 1
            return
        
        elif self.drone_z == self.height/2: 
            return

        sum_vx, sum_vy = 0.0, 0.0
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
            weight = 1.0/max(speed, 0.001)
            weights.append((weight, Vx, Vy))

        total_weight = sum(weight for weight, Vx, Vy in weights)
        if total_weight == 0: total_weight = 1.0

        avg_vx = sum(weight * Vx for weight, Vx, Vy in weights)/total_weight
        avg_vy = sum(weight * Vy for weight, Vx, Vy in weights)/total_weight

        sum_vx, sum_vy = 0.0, 0.0
        for weight, Vx, Vy in weights:
            norm_weight = weight / total_weight
            sum_vx += norm_weight * (Vx - avg_vx)
            sum_vy += norm_weight * (Vy - avg_vy)

        self.drone_dx = sum_vx
        self.drone_dy = sum_vy
        self.drone_dz = 1.0

        self.drone_x += self.drone_dx
        self.drone_y += self.drone_dy
        self.drone_z += self.drone_dz
        self.frame_count += 1

        for point in self.points:
            if point['z'] < self.drone_z: point['z'] += random.uniform(0, 200)

        transformed, velocities = [], []

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

            transformed.append({'num': point['num'], 'x': x_proj, 'y': y_proj, 'world_x': point['x'], 'world_y': point['y'], 'world_z': point['z']})
            velocities.append((Vx, Vy))

        os.system("clear")
        self.get_logger().info(f"Drone Position: x = {self.drone_x:.2f}, y = {self.drone_y:.2f}, z = {self.drone_z:.2f}\n")

        for i, point in enumerate(transformed):
            Vx, Vy = velocities[i]
            self.get_logger().info(f"Point {point['num']}: x = {point['world_x']:.2f}, y = {point['world_y']:.2f}, z = {point['world_z']:.2f}")
            self.get_logger().info(f"Drone Frame: x = {point['x']:.2f}, y = {point['y']:.2f}")
            self.get_logger().info(f"Velocity: Vx = {Vx:.2f}, Vy = {Vy:.2f}\n")
        
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