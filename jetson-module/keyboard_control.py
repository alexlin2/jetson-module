#!/usr/bin/python3
import rospy
from std_msgs import String
import keyboard

class Teleop():

    def __init__(self):
        rospy.loginfo('Starting keyboard controller')
        rospy.init_node('keyboard_capture')
        self.rate = rospy.Rate(30)

        self.msg = ' '
        self.pub_keypress = rospy.Publisher('keyboard_teleop', String, queue_size = 1)

    def capture_keyboard(self):
        keypress = ''
        if keyboard.is_pressed('w'):
            keypress += 'w'
        if keyboard.is_pressed('s'):
            keypress += 's'
        if keyboard.is_pressed('a'):
            keypress += 'a'
        if keyboard.is_pressed('d'):
            keypress += 'd'

        self.msg = keypress

    def run():
        while not rospy.is_shutdown():
            self.capture_keyboard()
            self.pub_keypress.publish(self.msg)
            self.rate.sleep()
            
if __name__ == "__main__":
    cmd = Teleop()
    cmd.run()


