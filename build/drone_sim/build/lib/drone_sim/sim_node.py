import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class RobotSim(Node):
    def __init__(self):
        super().__init__('robot_sim')

        # Simulation space
        self.grid_size = 20
        self.x = self.grid_size // 2
        self.y = self.grid_size // 2

        # Subscriptions and timer
        self.subscription = self.create_subscription(
            String,
            'cmd_move',
            self.move_callback,
            10
        )
        self.timer = self.create_timer(0.2, self.print_grid)

    def move_callback(self, msg):
        command = msg.data.strip().lower()
        if command == 'up' and self.y > 0:
            self.y -= 1
        elif command == 'down' and self.y < self.grid_size - 1:
            self.y += 1
        elif command == 'left' and self.x > 0:
            self.x -= 1
        elif command == 'right' and self.x < self.grid_size - 1:
            self.x += 1
        else:
            self.get_logger().info('Invalid or out-of-bounds command')

    def print_grid(self):
        # Clear the terminal cleanly
        print("\033[H\033[2J", end='')  # Move cursor home + clear screen

        for row in range(self.grid_size):
            line = ''
            for col in range(self.grid_size):
                if row == self.y and col == self.x:
                    line += '🟥'  # Robot shown as a solid red square
                else:
                    line += '   '  # Blank space
            print(line)
        print(f'\nRobot position: ({self.x}, {self.y})')
        print('Use ros2 topic pub /cmd_move std_msgs/String "data: \'right\'" etc.')

def main(args=None):
    rclpy.init(args=args)
    node = RobotSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
