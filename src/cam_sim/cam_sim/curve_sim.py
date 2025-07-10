import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib
import random
import numpy as np
import os
import math

matplotlib.use('Qt5Agg')

class CurvedSim(Node):
    def __init__(self):
        super().__init__('curved_sim')

        # Corridor and drone parameters
        self.height = 200
        self.bound_width = 60
        self.R = 150.0               # Radius of centerline curve
        self.arc_start = 0.0
        self.arc_end = np.pi / 2    # 90 degrees turn

        self.focal_length = 60

        # Drone state: position along curve (theta), lateral offset (radial), height
        self.theta = 0.0             # angle along curve centerline
        self.angular_speed = 0.008   # radians per frame (controls forward speed)
        self.drone_radial_offset = 0.0  # lateral offset from centerline (aim to control this to 0)
        self.drone_z = -self.height/2

        # Drone heading tangent to curve (computed each frame)
        self.drone_heading = self.theta + np.pi / 2

        self.frame_count = 0

        self.points = self.generate_points()
        self.transformed_points = []

        self.timer = self.create_timer(0.02, self.update)

    def generate_points(self):
        points = []
        point_count = 20

        left_radius = self.R - self.bound_width
        right_radius = self.R + self.bound_width

        for i in range(point_count):
            angle = random.uniform(self.arc_start, self.arc_end)
            z = random.uniform(-self.height/4, 0)
            x = left_radius * math.cos(angle)
            y = left_radius * math.sin(angle)
            points.append({'num': i, 'x': x, 'y': y, 'z': z})

        for i in range(point_count):
            angle = random.uniform(self.arc_start, self.arc_end)
            z = random.uniform(-self.height/4, 0)
            x = right_radius * math.cos(angle)
            y = right_radius * math.sin(angle)
            points.append({'num': i + point_count, 'x': x, 'y': y, 'z': z})

        return points

    def drone_position(self):
        # Drone position on centerline plus lateral offset (radial)
        cx = self.R * math.cos(self.theta)
        cy = self.R * math.sin(self.theta)

        # Radial direction vector (points outward from center)
        radial_dir_x = math.cos(self.theta)
        radial_dir_y = math.sin(self.theta)

        # Add lateral offset from centerline along radial direction
        drone_x = cx + self.drone_radial_offset * radial_dir_x
        drone_y = cy + self.drone_radial_offset * radial_dir_y

        return drone_x, drone_y, self.drone_z

    def drone_heading_vector(self):
        # Heading tangent to curve, perpendicular to radius vector
        heading = self.theta + np.pi / 2
        dx = math.cos(heading)
        dy = math.sin(heading)
        return dx, dy

    def point_relative_to_drone(self, px, py, pz, drone_x, drone_y, drone_z, drone_heading):
        # Vector from drone to point in global frame
        dx = px - drone_x
        dy = py - drone_y
        dz = pz - drone_z

        # Rotate vector into drone local frame by -heading
        cos_h = math.cos(-drone_heading)
        sin_h = math.sin(-drone_heading)

        local_x = cos_h * dx - sin_h * dy
        local_y = sin_h * dx + cos_h * dy
        local_z = dz

        return local_x, local_y, local_z

    def scale_vel(self, vel):
        return vel * 12 * abs(vel*100)**2.2

    def weight_vels(self, drone_x, drone_y, drone_z, drone_heading, drone_dx, drone_dy, drone_dz):
        left_vx_avg, right_vx_avg = 0.0, 0.0
        bottom_vy_avg, top_vy_avg = 0.0, 0.0

        total_weight = 0.0
        weights = []

        for point in self.points:
            # Get point relative to drone in drone frame
            local_x, local_y, local_z = self.point_relative_to_drone(point['x'], point['y'], point['z'],
                                                                    drone_x, drone_y, drone_z, drone_heading)

            if local_z <= 0:
                continue

            # Apparent velocities (optical flow) components
            Vx = (-drone_dx * local_z + drone_dz * local_x) / (local_z ** 2)
            Vy = (-drone_dy * local_z + drone_dz * local_y) / (local_z ** 2)

            speed = (Vx**2 + Vy**2)**0.5
            weight = speed**0.5

            weights.append((local_x, local_y, weight, Vx, Vy))
            total_weight += weight

        if total_weight == 0:
            total_weight = 1.0

        for local_x, local_y, weight, Vx, Vy in weights:
            if local_x < 0: 
                left_vx_avg += weight * Vx
            else: 
                right_vx_avg += weight * Vx

            if local_y < 0: 
                bottom_vy_avg += weight * Vy
            else: 
                top_vy_avg += weight * Vy

        left_vx_avg /= total_weight
        right_vx_avg /= total_weight
        bottom_vy_avg /= total_weight
        top_vy_avg /= total_weight

        # Update control inputs in drone frame
        drone_dx = (self.scale_vel(left_vx_avg) + self.scale_vel(right_vx_avg))
        drone_dy = (self.scale_vel(bottom_vy_avg) + self.scale_vel(top_vy_avg))
        drone_dz = 1.0

        return drone_dx, drone_dy, drone_dz, self.scale_vel(left_vx_avg), self.scale_vel(right_vx_avg)

    def update(self):
        plt.clf()

        if self.theta >= self.arc_end:
            print("Reached end of curved corridor")
            return

        drone_x, drone_y, drone_z = self.drone_position()
        drone_dx_old, drone_dy_old = 0.0, 0.0
        drone_dz = 1.0
        drone_heading = self.theta + np.pi / 2

        # Compute weighted apparent velocities and control corrections
        drone_dx, drone_dy, drone_dz, left_vx_avg, right_vx_avg = self.weight_vels(
            drone_x, drone_y, drone_z, drone_heading, drone_dx_old, drone_dy_old, drone_dz)

        # Update drone's lateral offset by drone_dy (local frame)
        # Here drone_dy controls radial offset from centerline
        self.drone_radial_offset += drone_dy * 0.2  # scale factor for smooth control

        # Advance drone along curve
        self.theta += self.angular_speed
        self.drone_z += drone_dz * 0.5  # forward speed along vertical axis (adjust as needed)

        # Update for plotting
        drone_x, drone_y, drone_z = self.drone_position()
        drone_heading = self.theta + np.pi / 2

        # Plot corridor walls (arcs)
        angles = np.linspace(self.arc_start, self.arc_end, 100)
        left_wall_x = (self.R - self.bound_width) * np.cos(angles)
        left_wall_y = (self.R - self.bound_width) * np.sin(angles)
        right_wall_x = (self.R + self.bound_width) * np.cos(angles)
        right_wall_y = (self.R + self.bound_width) * np.sin(angles)

        plt.plot(left_wall_x, left_wall_y, 'k-', linewidth=2)
        plt.plot(right_wall_x, right_wall_y, 'k-', linewidth=2)

        # Plot drone position
        plt.plot(drone_x, drone_y, 'ro', markersize=12)

        # Plot points on walls
        for point in self.points:
            plt.plot(point['x'], point['y'], 'bo', markersize=8)
            plt.text(point['x'], point['y'], str(point['num']), color='white', fontsize=7,
                     ha='center', va='center')

        # Plot centerline for reference
        center_x = self.R * np.cos(angles)
        center_y = self.R * np.sin(angles)
        plt.plot(center_x, center_y, 'g--', linewidth=1)

        # Control visualization arrows at drone position (in global frame)
        # Transform local drone velocity to global frame
        cos_h = math.cos(drone_heading)
        sin_h = math.sin(drone_heading)
        vel_global_x = cos_h * drone_dx - sin_h * drone_dy
        vel_global_y = sin_h * drone_dx + cos_h * drone_dy
        plt.quiver(drone_x, drone_y, vel_global_x*10, vel_global_y*10, color='red')

        # Limits and aspect
        plt.axis('equal')
        plt.xlim(self.R - self.bound_width*3, self.R + self.bound_width*3)
        plt.ylim(-self.bound_width*2, self.R + self.bound_width*3)
        plt.title(f"Drone theta={math.degrees(self.theta):.1f}°, lateral offset={self.drone_radial_offset:.2f}")
        plt.axis('off')

        plt.draw()
        plt.pause(0.01)

        # Console output
        os.system("clear")
        print(f"Drone position: x={drone_x:.2f}, y={drone_y:.2f}, z={drone_z:.2f}, lateral offset={self.drone_radial_offset:.3f}")
        print(f"Control signals: drone_dx={drone_dx:.3f}, drone_dy={drone_dy:.3f}, left_vx_avg={left_vx_avg:.3f}, right_vx_avg={right_vx_avg:.3f}")
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = CurvedSim()

    plt.ion()
    plt.show(block=False)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()