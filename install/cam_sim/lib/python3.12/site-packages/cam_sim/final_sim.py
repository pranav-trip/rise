import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib
import random
import numpy as np
import os

matplotlib.use('Qt5Agg')

class finalSim(Node):
    def __init__(self):
        super().__init__('final_sim')

        self.height = 200
        self.width = 200
        self.depth = 100
        self.bound_width = 60

        self.focal_length = 60

        self.drone_x, self.drone_z, self.drone_theta = 0.0, -self.height/2, 0.0
        self.drone_dx, self.drone_dz, self.drone_dt = 1.0, 1.0, 0.0

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

        if abs(self.drone_x) < self.bound_width/12: clr = 'green'
        else: clr = 'red'

        for side in [-1, 1]:
            x = side * self.bound_width/12
            plt.plot([x, x], [-self.height/2, self.height/2], color = clr, linestyle = '--')

        for point in self.points:
            plt.plot(point['x'], point['z'], 'bo', markersize=12)
            plt.plot(point['x'], point['z'], marker = f'${point["num"]}$', markersize = 7, color = 'white')
    
    def weight_vels(self, drone_x, drone_z):
        left_vx_avg, right_vx_avg = 0.0, 0.0
        vels = []
        max_vx, min_vx = -float('inf'), float('inf')

        theta = self.drone_theta
        R = np.array([
            [np.cos(theta), 0, -np.sin(theta)],
            [0.0, 1, 0.0 ],
            [np.sin(theta), 0, np.cos(theta)]
        ])

        drone_pos = np.array([drone_x, 0.0, drone_z])
        drone_vel = np.array([self.drone_dx, 0.0, self.drone_dz])
        angular_vel = np.array([0.0, self.drone_dt, 0.0])

        for point in self.points:
            point_pos = np.array([point['x'], 0.0, point['z']])

            p_rel = point_pos - drone_pos

            p_body = R.T @ p_rel
            dx, dy, dz = p_body

            v_rot = np.cross(angular_vel, p_body)

            v_body = v_rot + drone_vel
            vx_b, vy_b, vz_b = v_body

            Vx = - (vx_b * dz - vz_b * dx) / (dz ** 2)

            absVx = abs(Vx)
            if absVx > max_vx:
                max_vx = absVx
            if absVx < min_vx:
                min_vx = absVx

            vels.append({'dx': dx, 'Vx': Vx})

        for vel in vels:
            vel['Vx'] = (abs(vel['Vx']) - min_vx) / max((max_vx - min_vx), 0.001)
            if vel['dx'] < 0:
                vel['Vx'] *= -1

        for vel in vels:
            if vel['dx'] < 0:
                left_vx_avg += vel['Vx']
            else:
                right_vx_avg += vel['Vx']

        left_vx_avg /= max(1, self.point_count)
        right_vx_avg /= max(1, self.point_count)

        signal = (left_vx_avg + right_vx_avg) * 0.002

        return signal, left_vx_avg, right_vx_avg

    def control(self):
        signal, left_vx_avg, right_vx_avg = self.weight_vels(self.drone_x, self.drone_z)

        self.drone_dt = -signal
        self.drone_theta += self.drone_dt

        self.drone_dx = -np.sin(self.drone_theta)
        self.drone_dz = np.cos(self.drone_theta)

        self.drone_x += self.drone_dx
        self.drone_z += self.drone_dz

        plt.plot(self.drone_x, self.drone_z, 'ro', markersize=10)

        plt.plot(self.drone_x, self.drone_z, 'ro', markersize=12)
        #plt.plot(80, 0, 'ko', markersize=10)
        #plt.plot(-80, 0, 'ko', markersize=10)
        #plt.quiver(80, 0, 50*right_vx_avg, 0, angles='xy', scale_units='xy', scale=1, color='red')
        #plt.quiver(-80, 0, 50*left_vx_avg, 0, angles='xy', scale_units='xy', scale=1, color='red')

    def control_field(self):
        field_x = [-50, -20, 20, 50]
        field_z = [-75, -50, -25, 0, 25, 50, 75]

        for x in field_x:
            for z in field_z:
                if z < self.drone_z + self.height / 8:
                    scale = 10000
                    if (x, z) not in self.plotted:
                        signal, left_vx_avg, right_vx_avg = self.weight_vels(x, z)
                        plt.plot(x, z, 'bo', markersize=6)
                        plt.quiver(x, z, scale*signal, 8, angles='xy', scale_units='xy', scale=1, color='red')
                        self.plotted.append((x, z))
                        self.vels.append(signal)
                    
                    else:
                        index = self.plotted.index((x, z))
                        signal = self.vels[index]
                        plt.plot(x, z, 'bo', markersize=6)
                        plt.quiver(x, z, scale*signal, 8, angles='xy', scale_units='xy', scale=1, color='red')

    def test_control(self):

        wall_x, wall_z = -60, 20 #60, 45 and -60,70

        field_x = [-50, -20, 20, 50]
        field_z = [-75, -50, -25, 0, 25, 50, 75]

        for drone_x in field_x:
            for drone_z in field_z:
                if (drone_z > wall_z): continue

                dx = wall_x - drone_x
                dz = wall_z - drone_z
                vel = (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)
                vel = np.clip(vel, -0.5, 0.5)
                    
                plt.plot(drone_x, drone_z, 'bo', markersize=8)
                plt.plot(wall_x, wall_z, 'go', markersize=8)
                plt.quiver(drone_x, drone_z, 80 * vel, 10, angles='xy', scale_units='xy', scale=1, color='red')

    def transform(self):
        self.transformed_points = []

        for point in self.points:
            dx = point['x'] - self.drone_x
            dz = point['z'] - self.drone_z

            Vx = (-self.drone_dx * dz + self.drone_dz * dx) / (dz ** 2)
            self.transformed_points.append({'num': point['num'], 'x': point['x'], 'z': point['z'], 'Vx': Vx})

    def update(self):
        plt.clf()
        
        if self.drone_z == self.height/2: 
            return

        for point in self.points:
            if point['z'] < self.drone_z + self.height/4: 
                point['z'] = random.uniform(self.drone_z + self.height/4, self.drone_z + self.height/2)

        self.plot()
        self.control()
        self.control_field()
        self.transform()

        os.system("clear")
        print(f"Drone Position: x = {self.drone_x:.2f}, z = {self.drone_z:.2f}\n")

        for i, point in enumerate(self.transformed_points):
            print(f"Point {point['num']}: x = {point['x']:.2f}, z = {point['z']:.2f}")
            print(f"Velocity: V = {point['Vx']*1000:.2f}\n")

        plt.axis('equal')
        plt.xlim(-self.width/2, self.width / 2)
        plt.ylim(-self.height/2 , self.height / 2)
        plt.axis('off')
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = finalSim() 

    plt.ion()
    plt.show(block=False)
    fig = plt.gcf()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()