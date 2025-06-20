import rclpy
from rclpy.node import Node

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist

from pynput import keyboard

import sys

thrust_level = 50000
min_thrust=0
max_thrust=60000

class Thrust(Node):

    def __init__(self):
        global thrust_level
        if len(sys.argv) >= 2:
            thrust_level = sys.argv[1]
        super().__init__('thrust')

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
        
        self.publisher_ = self.create_publisher(Twist, '/cf2/cmd_vel_legacy', qos_profile)
        
        self.timer = self.create_timer(0.01, self.send_thrust)  # 50 Hz
        self.i=1
        
    def pose_get(self, msg):
        global thrust_level
        for pose in msg.poses:
            if pose.name == 'cf2':
            	#self.get_logger().info(f"I heard {pose.name}")
            	print(f"{thrust_level},{pose.pose.position.x},{pose.pose.position.y},{pose.pose.position.z},{pose.pose.orientation.x},{pose.pose.orientation.y},{pose.pose.orientation.z},{pose.pose.orientation.w}")
        #self.get_logger().info("New message")
        
    def send_thrust(self):
        global thrust_level
        #self.get_logger().info(f"Sending a thrust of {sys.argv[1]}")
        msg = Twist()
        msg.linear.x=0.0
        msg.linear.y=0.0
        if self.i%4 != 0:
            msg.linear.z=float(thrust_level)
        else:
            msg.linear.z=0.0
        msg.angular.z=0.0
        self.i = self.i + 1
        self.publisher_.publish(msg)


def on_press(key):
    global thrust_level
    try:
        if key == keyboard.Key.up:
            if thrust_level < max_thrust:
                thrust_level += 100
                #thrust.send_thrust(100.0)
                #print(f"Thrust increased to {thrust_level}")
        elif key == keyboard.Key.down:
            if thrust_level > min_thrust:
                thrust_level -= 100
                #thrust.send_thrust(100.0)
                #print(f"Thrust decreased to {thrust_level}")
        elif key == keyboard.Key.esc:
            # Stop listener
            print("Exiting...")
            return False
    except AttributeError:
        pass

def main(args=None):
    rclpy.init(args=args)
    
    input_thrust = 0

    thrust = Thrust()	
    
    with keyboard.Listener(on_press=on_press) as listener:
        rclpy.spin(thrust)
        listener.join()
    


    print("asdhasiudhsauid")

    
    
    print("sdasdasd")

    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thrust.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
