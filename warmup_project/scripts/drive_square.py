#!/usr/bin/env python

"""
"""

from __future__ import print_function, division #for python2 users

import rospy
import math
# from neato_node.msg import Bump #bump package
from geometry_msgs.msg import Twist, Vector3
# import for emergency stop with lost telometry
import atexit

class DriveSquareNode(object):
    def __init__(self):
        rospy.init_node('driveSquare')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # emergency stop:
        atexit.register(self.exit_handler)
        self.initial_time = 0.0
        self.current_time = rospy.get_rostime()
        # time spent going forward for 1m at .3 m/s
        self.time_forward = rospy.Duration(1/.3)
        # time turning for pi/2 rad at .3 rad/s
        self.time_turning = rospy.Duration(math.pi/.6)
        # linear and angular velocities
        self.linearVel = 0.0
        self.angularVel = 0.0

        
    def goForward(self):
        self.initial_time = rospy.get_rostime()
        # keep going for duration
        while self.current_time < (self.initial_time + self.time_forward) and not rospy.is_shutdown():
            self.linearVel = 0.3
            self.current_time = rospy.get_rostime()
            #print('linearVel: ' + str(self.linearVel))
            self.pub.publish(Twist(linear=Vector3(x=self.linearVel), angular=Vector3(z=self.angularVel)))
        # set back to 0 after desired time
        self.linearVel = 0.0

    def turn(self):
        self.initial_time = rospy.get_rostime()
        # keep going for duration
        while self.current_time < (self.initial_time + self.time_turning) and not rospy.is_shutdown():
            self.angularVel = 0.3
            self.current_time = rospy.get_rostime()
            #print('angularVel: ' + str(self.angularVel))
            self.pub.publish(Twist(linear=Vector3(x=self.linearVel), angular=Vector3(z=self.angularVel)))
        # set back to 0 after desired time
        self.angularVel = 0.0

    def driveSquare(self):
        for x in range(4):
            self.pub.publish(Twist(linear=Vector3(x=.1), angular=Vector3(z=0)))
            print("Going forward: " + str(x))
            self.goForward()
            print("Turning: " + str(x))
            self.turn()
        self.pub.publish(Twist(linear=Vector3(x=self.linearVel), angular=Vector3(z=self.angularVel)))

    def run(self):
        print("starting")
        print(str(self.time_forward))
        print(str(self.time_turning))
        self.driveSquare()

    def exit_handler(self):
        # emergency exit
        self.linearVel = 0.0
        self.angularVel = 0.0       

if __name__ == '__main__':
    node = DriveSquareNode()
    node.run()
