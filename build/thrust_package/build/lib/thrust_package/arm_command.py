import sys

import rclpy
from rclpy.node import Node

from crazyflie_interfaces.srv import Arm


class ArmClient(Node):

    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(Arm, 'cf2/arm')
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
    future = arm_client.request_arm(bool(int(sys.argv[1])))
    response = future.result()
    arm_client.get_logger().info(
        f'Result of arm: {response}')
    arm_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
