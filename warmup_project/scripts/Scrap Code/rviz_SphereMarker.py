#!/usr/bin/env python

"""
write ROS node that publishes 10 times a second
a message of type visualization_messages/Marker

specify to rviz to create a sphere at position x=1m
and y=2m in Neato's odometry coordinate system (odom)

"""

from __future__ import print_function

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA

class PublishSphereNode(object):
    def __init__(self):
        rospy.init_node('publish_sphere')
        self.pub = rospy.Publisher('/my_sphere', Marker, queue_size=10)
        # Set parameters
        # Pose:
        point = Point(1.0,2.0,1.0)
        quaternion = Quaternion(0,0,0,0)
        # Other things
        header = Header(stamp=rospy.Time.now(), frame_id="odom")
        ns = None
        id = None
        type = 2
        action = None
        scale = Vector3(1,1,1)
        color = ColorRGBA(255,105,180,100)
        points = None
        colors = None
        lifetime = rospy.Time(1,0)
        frame_locked = False
        text = None
        mesh_resource = None
        self.sphere = Marker(pose=Pose(position=point, orientation=quaternion), header=header, ns=ns, id=id, type=type, scale=scale, color=color, points=points,colors=colors,lifetime=lifetime,frame_locked=frame_locked,text=text,mesh_resource=mesh_resource)
        # can avoid line by formatting like sphere = Marker(), sphere.pose = Pose(etc) etc

    def run(self):
        r = rospy.Rate(.1) #10 Hz
        while not rospy.is_shutdown():
            self.pub.publish(self.sphere)

if __name__ == '__main__':
    node = PublishSphereNode()
    node.run()