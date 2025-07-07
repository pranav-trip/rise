import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib
import random
import os

matplotlib.use('Qt5Agg')

class VisualFlowSim(Node):
    def __init__(self):
        super().__init__('visual_flow_sim')

        self.boundary_x = 60
        self.plot_height = 200
        self.plot_width = 160

        self.focal_length = 60

        self.drone_x = 0.0
        self.drone_y = -self.plot_height / 2 + 10  # Start at bottom
        self.drone_dx = 0.0
        self.drone_dy = 0.0

        self.initial_boost_frames = 10  # move forward manually for this many frames
        self.frame_count = 0

        self.points = self.generate_random_points()

        self.timer = self.create_timer(0.02, self.update)
        self.last_printed = None

    def generate_random_points(self):
        points = []
        for i in range(5):
            y = random.uniform(-60, self.plot_height / 2 - 20)
            points.append({'id': i, 'x': -self.boundary_x, 'y': y})
        for i in range(5):
            y = random.uniform(-60, self.plot_height / 2 - 20)
            points.append({'id': 5 + i, 'x': self.boundary_x, 'y': y})
        return points

    def update(self):
        plt.clf()

        # Autonomous drone movement logic
        if self.frame_count < self.initial_boost_frames:
            self.drone_dy = 1
            return 0
            
        total_vx = 0.0
        total_vy = 0.0
        count = 0

        for p in self.points:
            dx = p['x'] - self.drone_x
            dy = p['y'] - self.drone_y

            if dy <= 0:
                continue  # ignore points behind

            # Compute apparent velocities
            Vx = (-self.drone_dx * dy + self.drone_dy * dx) / (dy ** 2)
            Vy = 0  # Only forward motion in camera frame
            total_vx += Vx
            total_vy += Vy
            count += 1

        if count > 0:
            # Simple gain to convert optical flow to movement
            gain = 10.0
            self.drone_dx = gain * total_vx
            self.drone_dy = 1.5  # constant forward motion

        self.drone_x += self.drone_dx
        self.drone_y += self.drone_dy
        self.frame_count += 1

        transformed = []
        velocities = []

        for p in self.points:
            dx = p['x'] - self.drone_x
            dy = p['y'] - self.drone_y

            if dy <= 0:
                x_proj = float('inf')
                y_proj = dy
                Vx = 0
            else:
                x_proj = self.focal_length * (dx / dy)
                y_proj = dy
                Vx = (-self.drone_dx * dy + self.drone_dy * dx) / (dy ** 2)

            transformed.append({'id': p['id'], 'x_proj': x_proj, 'y_proj': y_proj, 'world_x': p['x'], 'world_y': p['y']})
            velocities.append(Vx)

        # Console Output
        if transformed != self.last_printed:
            os.system("clear")
            self.get_logger().info(f'Drone Position: x = {self.drone_x:.2f}, y = {self.drone_y:.2f}\n')
            for i, p in enumerate(transformed):
                Vx = velocities[i]
                self.get_logger().info(f'Point {p["id"]}: x = {p["world_x"]:.2f}, y = {p["world_y"]:.2f}')
                self.get_logger().info(f'Drone Frame: x = {p["x_proj"]:.2f}, y = {p["y_proj"]:.2f}')
                self.get_logger().info(f'Velocity: Vx = {Vx:.2f}, Vy = 0.00\n')
            self.last_printed = transformed

        # Draw walls
        for side in [-1, 1]:
            x = side * self.boundary_x
            plt.plot([x]*2, [-self.plot_height/2, self.plot_height/2], 'k-')

        # Center line
        center_color = 'green' if abs(self.drone_x) < 1e-6 else 'red'
        plt.plot([0]*2, [-self.plot_height/2, self.plot_height/2], linestyle='--', color=center_color, linewidth=2)

        # Points
        for p in self.points:
            plt.plot(p['x'], p['y'], 'bo', markersize=12)
            plt.plot(p['x'], p['y'], marker=f'${p["id"]}$', markersize=7, color='white')

        # Drone
        plt.plot(self.drone_x, self.drone_y, 'ro', markersize=12)

        plt.axis('equal')
        plt.xlim(-self.plot_width / 2, self.plot_width / 2)
        plt.ylim(-self.plot_height / 2, self.plot_height / 2)
        plt.axis('off')
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = VisualFlowSim()

    plt.ion()
    plt.show(block=False)
    fig = plt.gcf()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
