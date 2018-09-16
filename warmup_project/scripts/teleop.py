#!/usr/bin/env python

# my own teleop code!

from __future__ import print_function # for python2 users

import rospy
# imports for keys
import tty
import select
import sys
import termios

# imports for actions
from geometry_msgs.msg import Twist, Vector3
# import for emergency stop with lost telometry
import atexit

# scaffholding:
# PublishAction obtains and interprets keyboard input in an angular and linear velocity for the Neato.
# This info is published to the 'cmd_vel' topic

class PublishAction(object):
    def __init__(self):
        rospy.init_node('teleop_node')
        # Process key code
        self.key = None
        self.settings = termios.tcgetattr(sys.stdin)
        # Publish Actions (velocities)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.linearVel = 0.0
        self.angularVel = 0.0
        # emergency stop:
        atexit.register(self.exit_handler)

    def interpretKey(self):
        # start with forward and backward
        if self.key == 'w':
            self.linearVel = 0.3
        elif self.key == 's':
            self.linearVel = -0.3
        elif self.key == 'd':
            self.angularVel = -0.3
        elif self.key == 'a':
            self.angularVel = 0.3
        elif self.key == 'f':
            self.angularVel = 0.0
        elif self.key == 'e':
            self.linearVel = 0.0
        else:
            self.linearVel = 0.0
            self.angularVel = 0.0


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run(self):
        print('WASD keys. E stops linear, F stops angular. All else cancels.')
        # Get key
        while self.key != '\x03': # unless Ctrl+C
            self.getKey()
            self.interpretKey() # sets velocity according to key
            # Publish action
            self.pub.publish(Twist(linear=Vector3(x=self.linearVel), angular=Vector3(z=self.angularVel)))
            print('Linear: ' + str(self.linearVel) + ', Angular: ' + str(self.angularVel))

    def exit_handler(self):
        # emergency exit
        self.linearVel = 0.0
        self.angularVel = 0.0

if __name__ == '__main__':
    node = PublishAction()
    node.run()

""" this is example code for getting keyboard input
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None

while key != '\x03':
    key = getKey()
    print key

# till here """

