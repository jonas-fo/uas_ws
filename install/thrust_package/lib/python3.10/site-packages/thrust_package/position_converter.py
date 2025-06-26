import rclpy
from rclpy.node import Node

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rclpy.clock import Clock	

import sys
import time


class Converter(Node):

    def __init__(self):
        super().__init__('converter')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.pose_get,
            qos_profile)
        self.subscription  # prevent unused variable warning
        
        self.posePublisher = self.create_publisher(PoseStamped, '/cf20/test_pose', qos_profile)
        
        

    def pose_get(self, msg):
        clock = Clock()
        for pose in msg.poses:
            if pose.name == 'cf20Clownfish':
                #self.get_logger().info(f"I heard {pose.name}")
                sendmsg = PoseStamped()
                sendmsg.header.stamp = clock.now().to_msg()
                sendmsg.header.frame_id = msg.header.frame_id
                sendmsg.pose.position.x = pose.pose.position.x
                sendmsg.pose.position.y = pose.pose.position.y
                sendmsg.pose.position.z = pose.pose.position.z
                sendmsg.pose.orientation.x = pose.pose.orientation.x
                sendmsg.pose.orientation.y = pose.pose.orientation.y
                sendmsg.pose.orientation.z = pose.pose.orientation.z
                sendmsg.pose.orientation.w = pose.pose.orientation.w
                self.posePublisher.publish(sendmsg)
                


def main(args=None):
    rclpy.init(args=args)
    
    input_thrust = 0

    converter = Converter()	
    

    rclpy.spin(converter)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
