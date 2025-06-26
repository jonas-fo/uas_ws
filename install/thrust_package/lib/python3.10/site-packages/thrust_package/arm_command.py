import sys

import rclpy
from rclpy.node import Node

from crazyflie_interfaces.srv import Arm

from inputs import get_gamepad
import time


class ArmClient(Node):

    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(Arm, 'cf20/arm')
        timer_period = 0.5  # seconds
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Arm.Request()

    def request_arm(self,a):
        self.req.arm = a
        return self.cli.call_async(self.req)


def main():
    rclpy.init()
    
    arm_client = ArmClient()
    loops = True
    arm = True
    while loops:
        try:
            events = get_gamepad()
            for event in events:
                if event.ev_type == 'Key':
                    if event.code == 'BTN_SOUTH':  # Assuming BTN_SOUTH is the button to arm/disarm
                        if event.state == 1:  # Button pressed
                            future = arm_client.request_arm(arm)
                            print(f"Requesting arm: {arm}")
                            response = future.result()
                            arm_client.get_logger().info(
                                f'Result of arm: {response}')
                            arm = not arm  # Toggle arm state

        except Exception as e:
            print(f"Error reading gamepad input: {e}")
            time.sleep(0.1)
    arm_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
