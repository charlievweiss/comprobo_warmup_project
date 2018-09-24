#!/usr/bin/env python

"""
write ROS node that publishes 10 times a second
a message of type visualization_messages/Marker

specify to rviz to create a sphere at position x=1m
and y=2m in Neato's odometry coordinate system (odom)

"""

from __future__ import print_function #for python2 users

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA

class PublishSphereNode(object):
    #takes scale (array of 3), color (array of 4) , and point (array of 3) and creates sphere marker at that location

    def __init__(self, scale, color, point):
        #rospy.init_node('publish_sphere')
        self.pub = rospy.Publisher('/markers', Marker, queue_size=10)
        # Set parameters
        # for Pose:
        point = Point(point[0],point[1], point[2])
        # Other things
        header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        type = 2
        scale = Vector3(scale[0],scale[1],scale[2])
        color = ColorRGBA(color[0],color[1],color[2],color[3])
        lifetime = rospy.Time(1,0)
        self.sphere = Marker(pose=Pose(position=point), header=header,type=type, scale=scale, color=color)
        # can avoid line by formatting like sphere = Marker(), sphere.pose = Pose(etc) etc

    def run(self):
        r = rospy.Rate(.1) #10 Hz
        while not rospy.is_shutdown():
            self.pub.publish(self.sphere)

if __name__ == '__main__':
    node = PublishSphereNode()
    node.run()