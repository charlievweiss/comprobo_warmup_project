#!/usr/bin/env python

from __future__ import print_function, division #for python2 users
import rospy
import numpy as np

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import atexit
import emergency_stop

#import create_marker

"""
It works!
"""

class ObstacleAvoidance(object):
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        # subscribe to laser scans
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # publish velocities
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # array to hold distances from laser scanner
        self.distances = np.ones(362)
        self.linearVel = 0.3 # start moving
        self.angularVel = 0.0

        # Emergency stops
        self.estop = emergency_stop.EmergencyStopNode()
        atexit.register(self.exit_handler)

    def process_scan(self, m):
        for x in range(len(self.distances)-1):
            if m.ranges[x] != 0.0:
                self.distances[x] = m.ranges[x]

    def find_max_distance(self, distances, start, end):
        # angles is array, start is first index, end is last index
        section = distances[start:end]
        max_distance = max(section)
        if max_distance > 15:
            max_distance = 0
        return max_distance

    def check_obstacle(self):
        if self.front_distance < 1.25:
            self.linearVel = 0.0
            self.angularVel = 1.0 # turns left by default. this may need changing
        else:
            self.linearVel = 0.3
            self.angularVel = 0.0

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # have to do this from 355-360 and 0-5 so it's a little weird:
            self.front_distance = (self.find_max_distance(self.distances, 355, 360)+self.find_max_distance(self.distances,0,5))/2

            # check for something in front
            self.check_obstacle()

            # Publish action
            self.pubVel.publish(Twist(linear=Vector3(x=self.linearVel), angular=Vector3(z=self.angularVel)))

    def exit_handler(self):
        # emergency exit
        self.linearVel = 0.0
        self.angularVel = 0.0
        print("thank you for shopping with obstacle_avoidance")

if __name__ == '__main__':
    node = ObstacleAvoidance()
    node.run()
    