#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import pygame
from pygame.locals import *

class Teleop():

    def __init__(self):
        rospy.loginfo('Starting keyboard controller')
        rospy.init_node('keyboard_capture')
        self.rate = rospy.Rate(30)
        pygame.init()
        window = pygame.display.set_mode((640,480),0,32)
        window.fill((0,0,0))
        self.msg = ' '
        self.pub_keypress = rospy.Publisher('keyboard_teleop', String, queue_size = 1)

    def capture_keyboard(self):
        key_pressed = pygame.key.get_pressed()
        keypress = '' 
        if key_pressed[K_w]:
            keypress += 'w'
        if key_pressed[K_s]:
            keypress += 's'
        if key_pressed[K_a]:
            keypress += 'a'
        if key_pressed[K_d]:
            keypress += 'd'

        print(keypress)

        self.msg = keypress

    def run(self):
        while not rospy.is_shutdown():
            self.capture_keyboard()
            self.pub_keypress.publish(self.msg)
            pygame.display.flip()
            pygame.event.pump()
            self.rate.sleep()
            
if __name__ == "__main__":
    cmd = Teleop()
    cmd.run()



