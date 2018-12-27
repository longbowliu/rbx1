#!/usr/bin/env python

""" nav_test.py - Version 1.1 2013-12-20

    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    On each round, select a new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import actionlib
import tf
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

class TestInitalpose():
    def __init__(self):
        rospy.init_node('test_initalpose', anonymous=False)
        rospy.loginfo("start test inital pose...")
    
        self.setpose_pub = rospy.Publisher("initialpose",PoseWithCovarianceStamped,latch=True, queue_size=1)
    
        #self.setpose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped,queue_size=10)
    
        self.set_pose = {'x':-0.170512974262,'y':-0.0195373892784,'a':0.0}
        self.test_set_pose_flag = True
        self.test_set_pose_cnt = 3
    
    
        while self.test_set_pose_flag == True:
    
            self.set_inital_pose()
            self.test_set_pose_cnt -= 1
            if self.test_set_pose_cnt == 0:
                self.test_set_pose_flag = False
        rospy.sleep(1)

    def set_inital_pose(self):
        # Define a set inital pose publisher.
        rospy.loginfo("start set pose...")
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.pose.position.x = self.set_pose['x']
        p.pose.pose.position.y = self.set_pose['y']
        p.pose.pose.position.z = self.set_pose['a']
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, self.set_pose['a'])
        p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0
    
        self.setpose_pub.publish(p)
if __name__ == '__main__':
    try:
        TestInitalpose()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")