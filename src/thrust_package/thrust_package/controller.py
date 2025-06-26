import rclpy
from rclpy.node import Node

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from crazyflie_interfaces.msg import Position
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rclpy.clock import Clock	

import sys
import time
from scipy.spatial.transform import Rotation as R

from inputs import get_gamepad

setpoint_position=[-2.44,-5.71,1.3]
current_pose=[0, 0, 0, 0 , 0 , 0, 1]

controller_on = False

integral_z=0
integral_y=0
integral_x=0

prev_x=0
prev_y=0
prev_z=0

offset_gain=0

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
        
        self.setpointsubscription = self.create_subscription(
            Position,
            '/cf20/cmd_position',
            self.setpoint_get,
            qos_profile)
        
        self.start_time= time.time()
        self.done = False
        
        self.posePublisher = self.create_publisher(Twist, '/cf20/cmd_vel_legacy', qos_profile2)
        self.setpointPublisher = self.create_publisher(Position, '/cf20/setpoint_position', qos_profile2)
        self.errorPublisher = self.create_publisher(Position, '/cf20/error', qos_profile2)


        self.timer = self.create_timer(0.01, self.zero_message)

        self.timer2 = self.create_timer(0.001, self.controller_listen)

        self.timer3 = self.create_timer(0.001, self.controller_send)

        self.timer4 = self.create_timer(0.01, self.publish_setpoint)

        self.timer5 = self.create_timer(0.01, self.publish_error)
        
    def zero_message(self):
        global integral_z
        global controller_on
        zero_message = Twist()
        zero_message.linear.x = 0.0
        zero_message.linear.y = 0.0
        zero_message.linear.z = 0.0
        zero_message.angular.x = 0.0
        zero_message.angular.y = 0.0
        zero_message.angular.z = 0.0
        if not controller_on:
            integral_z=0.0
            integral_y=0.0
            integral_x=0.0
            #print("Controller is off, sending zero message.")
            self.posePublisher.publish(zero_message)      
    '''
    def controller_listen(self):
        global controller_on
        global offset_gain
        try:
            events = get_gamepad()
            for event in events:
                if event.ev_type == 'Key':
                    if event.code == 'BTN_EAST':
                        if event.state == 1:
                            setpoint_position[1] += 0.05
                    if event.code == 'BTN_NORTH':
                        if event.state == 1:
                            setpoint_position[1] -= 0.05
                    if event.code == 'BTN_TR':
                        if event.state == 1:  # Button pressed
                            offset_gain += 100.0
                    if event.code == 'BTN_TL':
                        if event.state == 1:  # Button pressed
                            offset_gain -= 100.0
                    if event.code == 'BTN_WEST':
                        if event.state == 1:  # Button pressed
                            print("Toggling controller state...")
                            controller_on = not controller_on
        except Exception as e:
            print(f"Error reading gamepad input: {e}")
            time.sleep(0.1)
    '''

    def controller_listen(self):
        global controller_on
        global offset_gain
        try:
            events = get_gamepad()
            for event in events:
                if event.ev_type == 'Key':
                    if event.code == 'BTN_EAST':
                        if event.state == 1:
                            setpoint_position[1] += 0.05
                    if event.code == 'BTN_NORTH':
                        if event.state == 1:
                            setpoint_position[1] -= 0.05
                    if event.code == 'BTN_TR':
                        if event.state == 1:  # Button pressed
                            print(offset_gain)
                            offset_gain += 0.05
                    if event.code == 'BTN_TL':
                        if event.state == 1:  # Button pressed
                            print(offset_gain)
                            offset_gain -= 0.05
                    if event.code == 'BTN_WEST':
                        if event.state == 1:  # Button pressed
                            print("Toggling controller state...")
                            controller_on = not controller_on
                            self.start_time = time.time()
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
        global integral_z
        global integral_y
        global integral_x
        global prev_z
        global prev_y
        global prev_x
        global controller_on
        global offset_gain
        if not controller_on:
            #print("Controller is off, not processing pose data.")
            return
        #print("Controller is on, processing pose data...")
        offset_z = 43600.0
    
        send_message = Twist()
        
        z = current_pose[2]
        z_error = round(setpoint_position[2]-z,3)
        
        kp_z = 4000.0
        ki_z = 400.0
        kd_z = 400.0
        integral_z+=z_error*0.001
        velocity_z = (z-prev_z) / 0.001

        
        p_z = z_error*kp_z+integral_z*ki_z + -velocity_z*kd_z


        setpoint_yaw = 0.0
        q = [current_pose[3], current_pose[4], current_pose[5], current_pose[6]]
        r = R.from_quat(q)
        yaw = r.as_euler('xyz', degrees=True)[2]

        yaw_error = setpoint_yaw - yaw
        kp_yaw = -7.0
        p_yaw = yaw_error * kp_yaw

        x = current_pose[0]
        x_error = round(setpoint_position[0]-x,3)
        kp_x = 15.0
        ki_x = 0.0
        kd_x = 0.15 + offset_gain
        integral_x+=x_error*0.001

        derivative_x = (x_error - prev_x) / 0.001

        p_pitch = x_error*kp_x + integral_x*ki_x + derivative_x*kd_x
        
        y = current_pose[1]
        y_error = round(setpoint_position[1]-y,3)
        kp_y = -15.0
        ki_y = 0.0
        kd_y = 0.15 + offset_gain
        derivative_y = (y_error - prev_y) / 0.001
        integral_y+=y_error*0.001
        p_roll = y_error*kp_y + integral_y*ki_y + derivative_y*kd_y

        send_message.linear.x=max(min(p_pitch, 15.0), -15.0)
        send_message.linear.y= max(min(p_roll, 15.0), -15.0)
        send_message.linear.z=max(min(p_z + offset_z, 60000.0), 0.0)
        send_message.angular.x=0.0
        send_message.angular.y=0.0
        send_message.angular.z=p_yaw
        
        self.posePublisher.publish(send_message)
        prev_x= x_error
        prev_y= y_error
        prev_z= z

        if time.time() - self.start_time > 15 and not self.done:
            print("Moved")
            setpoint_position[0] += 1.0
            self.done = True
        
    def setpoint_get(self, msg):
    	
        setpoint_position[0] = msg.x
        setpoint_position[1] = msg.y
        setpoint_position[2] = msg.z

    def publish_setpoint(self):
        setpoint_msg = Position()
        setpoint_msg.x = setpoint_position[0]
        setpoint_msg.y = setpoint_position[1]
        setpoint_msg.z = setpoint_position[2]
        self.setpointPublisher.publish(setpoint_msg)

    def publish_error(self):
        error_msg = Position()
        error_msg.x = setpoint_position[0] - current_pose[0]
        error_msg.y = setpoint_position[1] - current_pose[1]
        error_msg.z = setpoint_position[2] - current_pose[2]
        self.errorPublisher.publish(error_msg)
        
    
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
