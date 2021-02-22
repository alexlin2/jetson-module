#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
import time

def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class ActuateControl():

    def __init__(self):
        rospy.loginfo("Message Type: uint16[], starting control node")
        rospy.init_node('control_vel')
        
        self._cmd_vel_msg = OverrideRCIn()
        self._cmd_input = Twist()
        self.rate = rospy.Rate(30)

        self.pub_cmd_vel = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 1)
        self.input_command = rospy.Subscriber('/robot/navigation/input', Twist, self.actuate_command)

        self._last_time_cmd_rcv = time.time()
        rospy.loginfo("Initialization complete")
    
    def actuate_command(self, msg):
        self._cmd_input = msg
        self._cmd_vel_msg.channels = [0,1500,0,1500,0,0,0,0]
        self._last_time_cmd_rcv = time.time()
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        steering = map(angular_z, -1.0, 1.0, 1000, 2000)
        throttle = map(linear_x, -1.0, 1.0, 1350, 1650)

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
                #rospy.loginfo("controller disconnected")
                self.set_cmd_vel_to_zero()
            self.rate.sleep()

if __name__ == "__main__":
    cmd = ActuateControl()
    cmd.run()