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

class WallFollowing(object):
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
        self.f_dis = 2.0 #front distance
        self.lf_dis = 2.0 #left front
        self.l_dis = 2.0 #left side
        self.lb_dis = 2.0 #left back
        self.rf_dis = 2.0 # right front
        self.r_dis = 2.0 # right side
        self.rb_dis = 2.0 # right back
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
            max_distance = 10
        return max_distance

    def check_obstacle(self):
        if self.f_dis < .75:
            self.linearVel = 0.0
            self.angularVel = .3 # turns left by default. this may need changing
            print('Look Out! Fdis: '+str(self.f_dis))
        elif self.f_dis < .5:
            self.linearVel = -.3
        else:
            self.linearVel = 0.3
            #self.angularVel = 0.0

    def make_horizontal(self):
        left_dif = self.lf_dis - self.lb_dis
        right_dif = self.rf_dis - self.rb_dis
        tolerance = .1
        if np.absolute(left_dif) < 6 or np.absolute(right_dif) < 6:
            if left_dif > tolerance or right_dif < -tolerance:
                #self.linearVel = 0.1
                self.angularVel = -.3
            elif left_dif < -tolerance or right_dif > tolerance:
                #self.linearVel = 0.1
                self.angularVel = .3
            else:
                #self.linearVel = 0.3
                self.angularVel = 0
        print('left_dif: '+str(left_dif)+' right_dif: '+str(right_dif)+' linearVel: '+str(self.linearVel))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # have to do this from 355-360 and 0-5 so it's a little weird:
            self.f_dis = (self.find_max_distance(self.distances, 355, 360))#+self.find_max_distance(self.distances,0,5))/2
            self.lf_dis = self.find_max_distance(self.distances,310,320)
            self.lb_dis = self.find_max_distance(self.distances,220,230)
            self.rf_dis = self.find_max_distance(self.distances,40,50)
            self.rb_dis = self.find_max_distance(self.distances,130,140)

            # check for something in front
            self.check_obstacle()
            self.make_horizontal()
            
            # Publish action
            print(self.linearVel)
            self.pubVel.publish(Twist(linear=Vector3(x=self.linearVel), angular=Vector3(z=self.angularVel)))

    def exit_handler(self):
        # emergency exit
        self.linearVel = 0.0
        self.angularVel = 0.0
        print("thank you for shopping with obstacle_avoidance")

if __name__ == '__main__':
    node = WallFollowing()
    node.run()
    