import rclpy
from rclpy.node import Node

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rclpy.clock import Clock	

import sys
import time
from scipy.spatial.transform import Rotation as R

from inputs import get_gamepad

setpoint_position=[-2.3,-6.108,1.3]
current_pose=[0, 0, 0, 0 , 0 , 0, 1]

controller_on = False

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        qos_profile2 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/cf20/test_pose',
            self.pose_get,
            qos_profile)
        '''
        self.setpointsubscription = self.create_subscription(
            PoseStamped,
            '/cf20/cmd_position',
            self.setpoint_get,
            qos_profile)
        self.subscription  # prevent unused variable warning
        '''
        
        self.posePublisher = self.create_publisher(Twist, '/cf20/cmd_vel_legacy', qos_profile2)

        self.timer = self.create_timer(0.01, self.zero_message)

        self.timer2 = self.create_timer(0.001, self.controller_listen)

        self.timer3 = self.create_timer(0.01, self.controller_send)
        
    def zero_message(self):
        global controller_on
        zero_message = Twist()
        zero_message.linear.x = 0.0
        zero_message.linear.y = 0.0
        zero_message.linear.z = 0.0
        zero_message.angular.x = 0.0
        zero_message.angular.y = 0.0
        zero_message.angular.z = 0.0
        if not controller_on:
            #print("Controller is off, sending zero message.")
            self.posePublisher.publish(zero_message)      

    def controller_listen(self):
        global controller_on
        try:
            events = get_gamepad()
            for event in events:
                if event.ev_type == 'Key':
                    if event.code == 'BTN_WEST':
                        if event.state == 1:  # Button pressed
                            print("Toggling controller state...")
                            controller_on = not controller_on
        except Exception as e:
            print(f"Error reading gamepad input: {e}")
            time.sleep(0.1)

    def pose_get(self,msg):
        current_pose[0] = msg.pose.position.x
        current_pose[1] = msg.pose.position.y
        current_pose[2] = msg.pose.position.z
        current_pose[3] = msg.pose.orientation.x
        current_pose[4] = msg.pose.orientation.y
        current_pose[5] = msg.pose.orientation.z
        current_pose[6] = msg.pose.orientation.w

    def controller_send(self):
        global controller_on
        if not controller_on:
            #print("Controller is off, not processing pose data.")
            return
        #print("Controller is on, processing pose data...")
        offset_z = 44000.0
    
        send_message = Twist()
        
        z = current_pose[2]
        z_error = setpoint_position[2]-z
        
        kp_z = 1000.0
        p_z = z_error*kp_z


        setpoint_yaw = 0.0
        q = [current_pose[3], current_pose[4], current_pose[5], current_pose[6]]
        r = R.from_quat(q)
        yaw = r.as_euler('xyz', degrees=True)[2]

        yaw_error = setpoint_yaw - yaw
        kp_yaw = -5.0
        p_yaw = yaw_error * kp_yaw

        x = current_pose[0]
        x_error = setpoint_position[0]-x
        kp_x = 10.0
        p_pitch = x_error*kp_x
        
        y = current_pose[1]
        y_error = setpoint_position[1]-y
        kp_y = -10.0
        p_roll = y_error*kp_y

        send_message.linear.x=p_pitch
        send_message.linear.y=p_roll
        send_message.linear.z=p_z + offset_z
        send_message.angular.x=0.0
        send_message.angular.y=0.0
        send_message.angular.z=p_yaw
        
        self.posePublisher.publish(send_message)
    '''    
    def setpoint_get(self, msg):
        setpoint_position = []
    '''
def main(args=None):
    rclpy.init(args=args)

    #input("Press Enter to start the controller...")
    initial_message = Twist()
    initial_message.linear.x = 0.0
    initial_message.linear.y = 0.0
    initial_message.linear.z = 0.0
    initial_message.angular.x = 0.0
    initial_message.angular.y = 0.0
    initial_message.angular.z = 0.0
    
    controller = Controller()

    controller.posePublisher.publish(initial_message)    

    rclpy.spin(controller)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
