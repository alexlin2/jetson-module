#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs import String
import time

class ManualControl():
    
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


