#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
import time

def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class ManualControl():

    def __init__(self):
        rospy.loginfo("Message Type: uint16[], starting control node")
        rospy.init_node('control_vel')
        
        self._cmd_vel_msg = OverrideRCIn()
        self._steer_cache = 0.0
        self._throttle_cache = 0.0

        self.rate = rospy.Rate(30)
        self.pub_cmd_vel = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)
        self.keyboard_cmd = rospy.Subscriber('keyboard_teleop', String, self.send_teleop_command)

        self._last_time_cmd_rcv = time.time()
        rospy.loginfo("Initialization complete")

    def send_teleop_command(self, msg):
        self._last_time_cmd_rcv = time.time()
        self._cmd_vel_msg.channels = [0,1500,0,1500,0,0,0,0]
        steer_increment = 0.05
        throttle_increment = 0.03

        if 's' in msg.data:
            if self._throttle_cache > 0:
                self._throttle_cache = 0.0
            else:
                self._throttle_cache -= throttle_increment
        elif 'w' in msg.data:
            if self._throttle_cache < 0:
                self._throttle_cache = 0.0
            else:
                self._throttle_cache += throttle_increment
        else: 
            self._throttle_cache = 0.0
        if 'a' in msg.data:
            if self._steer_cache > 0:
                self._steer_cache = 0.0
            else:
                self._steer_cache -= steer_increment
        elif 'd' in msg.data:
            if self._steer_cache < 0:
                self._steer_cache = 0.0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0

        self._steer_cache = min(1.0, max(-1.0, self._steer_cache))
        self._throttle_cache = min(1.0, max(-1.0, self._throttle_cache))
        steering = map(self._steer_cache, -1.0, 1.0, 1000, 2000)
        throttle = map(self._throttle_cache, -1.0, 1.0, 1350, 1650)
        self._cmd_vel_msg.channels[1] = throttle
        self._cmd_vel_msg.channels[3] = steering

        self.pub_cmd_vel.publish(self._cmd_vel_msg)
        
    def set_cmd_vel_to_zero(self):
        self._cmd_vel_msg.channels = [0,1500,0,1500,0,0,0,0]
        self.pub_cmd_vel.publish(self._cmd_vel_msg)
        
    def is_controller_connected(self):
        return time.time() - self._last_time_cmd_rcv < 3

    def run(self):

        while not rospy.is_shutdown():
            if not self.is_controller_connected():
                rospy.loginfo("controller disconnected")
                self.set_cmd_vel_to_zero()
            self.rate.sleep()

if __name__ == "__main__":
    cmd = ManualControl()
    cmd.run()
    
        



'''class ManualControl():
    
    def __init__(self):
        rospy.loginfo("Message Type: Twist, starting control node")
        rospy.init_node('control_vel')

        self._cmd_vel_msg = Twist()
        self._cmd_vel_msg.linear.y = 0
        self._cmd_vel_msg.linear.z = 0
        self._cmd_vel_msg.angular.x = 0
        self._cmd_vel_msg.angular.y = 0

        self.rate = rospy.Rate(30)
        self.pub_cmd_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        self.keyboard_cmd = rospy.Subscriber('keyboard_teleop', String, self.send_command)

        self._last_time_cmd_rcv = time.time()

        rospy.loginfo("Initialization complete")

    def send_command(self, msg):
        self._last_time_cmd_rcv = time.time()

        if 'w' in msg.data:
            self._cmd_vel_msg.linear.x = 0.3
        elif 's' in msg.data:
            self._cmd_vel_msg.linear.x = -0.3
        else: 
            self._cmd_vel_msg.linear.x = 0
        if 'a' in msg.data:
            self._cmd_vel_msg.angular.z = 0.5
        elif 'd' in msg.data:
            self._cmd_vel_msg.angular.z = -0.5
        else:
            self._cmd_vel_msg.angular.z = 0

        self.pub_cmd_vel.publish(self._cmd_vel_msg)
        
    def set_cmd_vel_to_zero():
        self._cmd_vel_msg.linear.x = 0
        self._cmd_vel_msg.angular.z = 0
        self.pub_cmd_vel.publish(self._cmd_vel_msg)
        
    def is_controller_connected(self):
        return time.time() - self._last_time_cmd_rcv < 3

    def run(self):

        while not rospy.is_shutdown():
            if not self.is_controller_connected():
                rospy.loginfo("controller disconnected")
                self.set_cmd_vel_to_zero()
            self.rate.sleep()

if __name__ == "__main__":
    cmd = ManualControl()
    cmd.run()


'''
