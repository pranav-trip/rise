import rclpy
from rclpy.node import Node
from djitellopy import Tello
import time

class testTello(Node):
    def __init__(self):
        super().__init__('testTello')
        self.tello = Tello()
        self.tello.connect(wait_for_state=False)
        print('Connected')

        self.timer = self.create_timer(2.0, self.test)

    def test(self):
        time.sleep(1)
        self.tello.takeoff()
        time.sleep(1)
        self.tello.flip_forward()
        time.sleep(1)
        self.tello.land()
        self.tello.end()
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = testTello()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()