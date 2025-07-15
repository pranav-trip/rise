import rclpy
from rclpy.node import Node
import os
import random
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np

class camSim(Node):
    def __init__(self):
        super().__init__('cam_sim')

        self.x_lim, self.y_lim, self.z_lim = 80, 50, 50

        self.drone_pos = np.array([self.x_lim/2, self.y_lim/2, self.z_lim/5])
        self.drone_vel = np.zeros(3)
        self.drone_yaw = 0.0
        self.drone_pitch = 0.0
        self.angular_velocity = np.zeros(3)

        self.show_vel = False
        self.vel_count = 0

        self.world_points = [
            np.array([random.uniform(0, self.x_lim), 
                      random.uniform(0, self.y_lim),
                      random.uniform(0, self.z_lim)]) 
            for x in range(10)
        ]

        self.focal_length = self.z_lim
        self.timer = self.create_timer(0.005, self.update)
        self.last_printed = None

    def key_control(self, event):
        lin_step = 0.2
        ang_step = 0.01

        if event.key == 'right': self.angular_velocity[1] -= ang_step
        elif event.key == 'left': self.angular_velocity[1] += ang_step
        if event.key == 'up': self.angular_velocity[0] += ang_step
        elif event.key == 'down': self.angular_velocity[0] -= ang_step

        if event.key == "i":  self.drone_vel[2] -= lin_step
        elif event.key == "o":  self.drone_vel[2] += lin_step

        if event.key == 'a': self.drone_vel[0] -= lin_step
        elif event.key == 'd': self.drone_vel[0] += lin_step
        if event.key == 'w': self.drone_vel[1] += lin_step
        elif event.key == 's': self.drone_vel[1] -= lin_step

        if event.key == "v": self.vel_count  += 1
        self.show_vel = (self.vel_count % 2 == 1)

    def update(self):
        plt.clf()

        self.drone_yaw -= self.angular_velocity[1]
        self.drone_pitch -= self.angular_velocity[0]

        R_yaw = np.array([
            [np.cos(self.drone_yaw), 0, np.sin(self.drone_yaw)],
            [0, 1, 0],
            [-np.sin(self.drone_yaw), 0, np.cos(self.drone_yaw)]
        ])

        R_pitch = np.array([
            [1, 0, 0],
            [0, np.cos(self.drone_pitch), -np.sin(self.drone_pitch)],
            [0, np.sin(self.drone_pitch), np.cos(self.drone_pitch)]
        ])

        R = R_yaw @ R_pitch

        self.transformed_points, self.velocities = [], []

        for point in self.world_points:
            x_proj, y_proj, dz, Vx, Vy = self.transform(point, R)
            self.transformed_points.append((x_proj, y_proj, dz))
            self.velocities.append((Vx, Vy))

        if self.last_printed != self.transformed_points:
            os.system("clear")
            self.get_logger().info(f'Drone Pos: {self.drone_pos}, Yaw: {self.drone_yaw:.2f}, Pitch: {self.drone_pitch:.2f}')

            for i, (x, y, z) in enumerate(self.transformed_points):
                Vx, Vy = self.velocities[i]
                wp = self.world_points[i]
                self.get_logger().info(f'Point {i}: world = {wp}, frame = ({x:.2f}, {y:.2f}, {z:.2f}), velocity = ({Vx:.2f}, {Vy:.2f})\n')

            self.last_printed = self.transformed_points

        for i, (x, y, _) in enumerate(self.transformed_points):
            plt.plot(x, y, 'bo', markersize=12)
            plt.plot(x, y, marker=f'${i}$', markersize=7, color='white')

            Vx, Vy = self.velocities[i]
            if self.show_vel:
                plt.quiver(x, y, 200*Vx, 200*Vy, angles='xy', scale_units='xy', scale=0.5, color='red')

        self.drone_pos += self.drone_vel
        self.drone_vel = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        plt.plot(0, 0, 'ko', markersize=12)
        plt.xlim(-self.focal_length, self.focal_length)
        plt.ylim(-self.focal_length, self.focal_length)
        plt.axis("off")
        plt.draw()
        plt.pause(0.005)

    def transform(self, point, R):
        p_rel = point - self.drone_pos
        p_body = R.T @ p_rel

        dx, dy, dz = p_body

        if dz <= 0:
            return float('inf'), float('inf'), dz, 0, 0

        x_proj = self.focal_length * (dx / dz)
        y_proj = self.focal_length * (dy / dz)

        angular_v = np.array([self.angular_velocity[0], self.angular_velocity[1], 0.0])
        v_rot = np.cross(angular_v, p_body)
        v_body = v_rot + R.T @ self.drone_vel

        Vx = - (v_body[0]*dz - v_body[2]*dx) / dz**2
        Vy = - (v_body[1]*dz - v_body[2]*dy) / dz**2

        return x_proj, y_proj, dz, Vx, Vy


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
