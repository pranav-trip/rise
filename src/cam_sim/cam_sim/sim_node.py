import rclpy
from rclpy.node import Node
import os
import random
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

class camSim(Node):
    def __init__(self):
        super().__init__('cam_sim')
    
        self.x_lim, self.y_lim, self.z_lim = 80, 50, 50

        self.drone_x , self.drone_y, self.drone_z = self.x_lim/2, self.y_lim/2, self.z_lim/5
        self.drone_dx, self.drone_dy, self.drone_dz = 0, 0, 0

        self.show_vel = False
        self.vel_count = 0

        self.world_points = [
            (random.uniform(0, self.x_lim), 
             random.uniform(0, self.y_lim),
             random.uniform(0, self.z_lim)) 
             for x in range(10)]

        self.focal_length = self.z_lim
        self.timer = self.create_timer(0.005, self.update)
        self.last_printed = None
    
    def key_control(self, event):
        step = 0.2
        
        if event.key == 'right': self.drone_dx += step
        elif event.key == 'left': self.drone_dx -= step
        
        if event.key == 'up': self.drone_dy += step
        elif event.key == 'down': self.drone_dy -= step
        
        if event.key == "i":  self.drone_dz += step
        elif event.key == "o":  self.drone_dz -= step
        
        if event.key == "v": self.vel_count  += 1
        if (self.vel_count%2 == 1): self.show_vel = True
        else: self.show_vel = False

    def update (self):
        plt.clf()

        self.transformed_points, self.velocities = [], []

        for x, y, z, in self.world_points:
            x_proj, y_proj, dz, Vx, Vy = self.transform(x, y, z)
            self.transformed_points.append((x_proj, y_proj, dz))
            self.velocities.append((Vx, Vy))
                
        if self.last_printed != self.transformed_points:
            os.system("clear")
            self.get_logger().info(f'Drone Position: x = {self.drone_x+self.drone_dx:.2f}, y = {self.drone_y+self.drone_dy:.2f}, z = {self.drone_z+self.drone_dz:.2f}\n')

            for i, (x, y, z) in enumerate(self.transformed_points):
                x_org, y_org, z_org = self.world_points[i]
                Vx, Vy = self.velocities[i]

                self.get_logger().info(f'Point {i}: x = {x_org:.2f}, y = {y_org:.2f}, z = {z_org:.2f}')
                self.get_logger().info(f'Drone Frame: x = {x:.2f}, y = {y:.2f}, z = {z:.2f}')
                self.get_logger().info(f'Velocity: Vx = {Vx:.2f}, Vy = {Vy:.2f}\n')
            
            self.last_printed = self.transformed_points

        for i, (x, y, z) in enumerate(self.transformed_points):    
            plt.plot(x, y, 'bo', markersize=12)
            plt.plot(x, y, marker=f'${i}$', markersize=7, color='white') 

            Vx, Vy = self.velocities[i]
            if (self.show_vel):
                plt.quiver(x, y, 200*Vx, 200*Vy, angles='xy', scale_units='xy', scale=1, color='red', headwidth=2, headlength=3)

        self.drone_x += self.drone_dx
        self.drone_y += self.drone_dy
        self.drone_z += self.drone_dz

        self.drone_dx = 0
        self.drone_dy = 0
        self.drone_dz = 0

        plt.plot(0, 0, 'ko', markersize=12)
        plt.xlim(-self.focal_length, self.focal_length)
        plt.ylim(-self.focal_length, self.focal_length)


        plt.axis("off")
        plt.draw()
        plt.pause(0.005)

    def transform(self, x, y, z):
        dx = x - self.drone_x
        dy = y - self.drone_y
        dz = z - self.drone_z

        if dz <= 0:
           return float('inf'), float('inf'), dz, 0, 0

        x_proj = self.focal_length * (dx/dz)
        y_proj = self.focal_length * (dy/dz)

        Vx = (-self.drone_dx*dz + self.drone_dz*dx) / (dz**2)
        Vy = (-self.drone_dy*dz + self.drone_dz*dy) / (dz**2)

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