import rclpy
from rclpy.node import Node
from djitellopy import Tello
import time

class testTello(Node):
    def __init__(self):
        super().__init__('testTello')
        self.tello = Tello()
        self.tello.connect()
        print('Connected\n')

        self.timer = self.create_timer(2.0, self.test)

    def test(self):
        time.sleep(1)
        print("Calibrated")
        battery = self.tello.get_battery()
        print(f"Battery: {battery}")
        self.tello.takeoff()
        time.sleep(2)
        self.tello.send_rc_control(0, 0, 0, 50)
        time.sleep(3)
        self.tello.land()
        self.tello.end()
        self.timer.cancel()
        print("Finished")

def main(args=None):
    rclpy.init(args=args)
    node = testTello()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()