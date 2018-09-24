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
Oh christ

CHARLIE: LET GO OF VISUALIZATION, JUST TRY TO GET THE OTHER PARTS WORKING


"""

class WallFollowing(object):
    def __init__(self):
        rospy.init_node('wall_following')
        # subscribe to laser scans
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # publish velocities
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # publish markers
        self.pubMark = rospy.Publisher('/markers', Marker, queue_size=10)
        # array to hold distances from laser scanner
        self.distances = np.ones(362)
        self.linearVel = 0.0 # start moving
        self.angularVel = 0.0
        self.front_distance = 1.0 # approx directly in front of neato
        self.left_front_distance = 1.0 # approx 315 deg
        self.left_back_distance = 1.0 # approx 225 deg
        self.right_front_distance = 1.0 # approx 315 deg
        self.right_back_distance = 1.0 # approx 225 deg

        self.left_front_angle = 360 # QUESTION
        self.left_back_angle = 0 # QUESTION
        # ARROW MARKERS
        self.front_marker = Marker()
        self.left_front_marker = Marker()
        self.left_back_marker = Marker()
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
        if self.front_distance < .5:
            self.angularVel = 0.5 # turns left by default. this may need changing
            self.linearVel = 0.0

    def make_horizontal(self):
        if left_back_distance > 0 and left_front_distance > 0:
            self.left_front_angle = np.degrees(np.arctan(self.left_back_distance/self.left_front_distance))
            self.left_back_angle = np.degrees(np.arctan(self.left_front_distance/self.left_back_distance))
            angle_difference = self.left_front_angle-self.left_back_angle
            if self.left_front_distance < .5 or self.left_back_distance < .5:
                # MAY NEED TO CHANGE PROPORTION
                self.angularVel = 0.3*angle_difference # turns left if positive, right if negative
            else:
                self.angularVel = 0.0
        else:
            angularVel = 0.0
        

    def visualize_points(self,deg,distance):
        deg = np.radians(deg)
        scale = [.1,.1,.1]
        color = [255,105,180,100]
        point = [distance*np.cos(deg),distance*np.sin(deg),0]
        return scale, color, point

    def visualize_arrow(self, deg, distance, id):
        #if distance > 0:
        header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        type = 2
        #color = ColorRGBA(255,105,180,100)
        color = ColorRGBA(1,1,0,1)
        #point = Point((distance*np.cos(deg)),(distance*np.sin(deg)),0)
        x = distance*np.cos(deg)
        y = distance*np.sin(deg)
        point = Point(x,y,0)
        #scale = Vector3(.01,.05,distance) # arrow?
        scale = Vector3(.1,.1,.1) # sphere
        lifetime = rospy.Time(1,0)
        marker = Marker(pose=Pose(position=point), header=header, type=type, color=color, scale=scale, lifetime=lifetime, id=id)
        self.pubMark.publish(marker)
        #return marker


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # have to do this from 355-360 and 0-5 so it's a little weird:
            self.front_distance = (self.find_max_distance(self.distances, 355, 360)+self.find_max_distance(self.distances,0,5))/2
            self.left_front_distance = self.find_max_distance(self.distances,310,320)
            self.left_back_distance = self.find_max_distance(self.distances,220,230)
            self.right_front_distance = self.find_max_distance(self.distances,40,50)
            self.right_back_distance = self.find_max_distance(self.distances,130,140)
            # check for something in front
            self.check_obstacle()
            # orient appropriately
            #self.make_horizontal()
            # Publish action
            self.pubVel.publish(Twist(linear=Vector3(x=self.linearVel), angular=Vector3(z=self.angularVel)))
            # ARROW MARKERS
            self.front_marker = self.visualize_arrow(0, self.front_distance, 1)
            self.left_front_maker = self.visualize_arrow(315, self.left_front_distance, 2)
            self.left_back_marker = self.visualize_arrow(225, self.left_back_distance, 3)
            self.right_front_marker = self.visualize_arrow(45, self.right_front_distance, 4)
            self.right_back_marker = self.visualize_arrow(135, self.right_back_distance, 5)
            """self.pubMark.publish(self.front_marker)
            self.pubMark.publish(self.left_front_marker)
            self.pubMark.publish(self.left_back_marker)
            self.pubMark.publish(self.right_front_marker)
            self.pubMark.publish(self.right_back_marker)"""


            # Troubleshooting
            #print("front: " + str(self.front_distance) + " left front: " + str(self.left_front_distance) + " left back: " + str(self.left_back_distance))
            #print("right front: " + str(self.right_front_distance) + " right back: " + str(self.right_back_distance))


            # THIS COULD BE THE END OF THE CODE
            """self.front_marker = self.visualize_arrow(0, self.front_distance)
            self.left_front_distance = self.visualize_arrow(315, self.left_front_distance)
            self.left_back_distance = self.visualize_arrow(225, self.left_back_distance)
            # PUBLISH ARROWS
            self.pubMark.publish(self.front_marker)
            self.pubMark.publish(self.left_front_marker)
            self.pubMark.publish(self.left_back_distance)"""
            #Bump stop
            #self.estop.run()

    def exit_handler(self):
        # emergency exit
        self.linearVel = 0.0
        self.angularVel = 0.0
        print("thank you for shopping with wall_following")

if __name__ == '__main__':
    node = WallFollowing()
    node.run()
    