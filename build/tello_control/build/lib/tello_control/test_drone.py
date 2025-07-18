import rclpy
from rclpy.node import Node
from djitellopy import Tello
import time

class testTello(Node):
    def __init__(self):
        super().__init__('testTello')
        self.tello = Tello()
        self.tello.connect(wait_for_state=False)
        print('connected!')

        self.timer = self.create_timer(2.0, self.test)

    def test(self):
        self.timer.cancel()
        self.tello.takeoff()

        self.tello.send_rc_control(0, 40, 0, 0)
        time.sleep(2)
        self.tello.send_rc_control(0, 0, 0, 0)
        time.sleep(2)
        self.tello.flip_forward()

        self.tello.land()
        self.tello.end()

def main(args=None):
    rclpy.init(args=args)
    node = testTello()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()