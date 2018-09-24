#!/usr/bin/env python

import rospy
import create_marker

class Test():
    def __init__(self):
        self.scale = [1,1,1]
        self.color = [255,105,180,100]
        self.point = [1,2,1]

    def run(self):
        while not rospy.is_shutdown():
            create_marker.PublishSphereNode(self.scale,self.color,self.point).run()

if __name__ == '__main__':
    node = Test()
    node.run()