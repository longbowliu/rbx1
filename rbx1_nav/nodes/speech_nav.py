#!/usr/bin/env python
# -*- coding:UTF-8 -*-

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
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist,PoseStamped

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from std_msgs.msg import String
from nav_msgs.msg import Path
from rbx1_nav.msg import myPath
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.locations = dict()
        '''
        走廊门口：-3.2529；-3.98846;0  0;0；-0.23051;0.97307
        休息室：7.5777；-5.6911;9  0;0;0.19;0.98178
        大会议室：5.4494;-1.7682;0  0;0;0.0298;0.9995
        机房： -4.5465;0.5764;0  0;0;0.8645;0.50263
        回来：0.12243,0.0079,0  0,0,-0.2534,0.9673
        '''
        self.locations['small_meeting'] = Pose(Point(0.95936,1.8196,0.0), Quaternion(0.000, 0.000, 0.22139,0.97519))
        self.locations['rest'] = Pose(Point(7.5777, -5.6911, 0.000), Quaternion(0.000, 0.000, 0.19,0.98178))
        self.locations['hall_bedroom'] = Pose(Point(-3.719, 4.401, 0.000), Quaternion(0.000, 0.000, 0.733, 0.680))
        self.locations['big_meeting'] = Pose(Point(5.4494,-1.7682, 0.000), Quaternion(0.000, 0.000, 0.0298,0.9995))
        self.locations['jifang'] = Pose(Point(-4.5465,0.5764, 0.000), Quaternion(0.000, 0.000, 0.8645,0.50263))
        self.locations['comeback'] = Pose(Point(0.12243,0.0079,0), Quaternion( 0,0,-0.2534,0.9673))
        
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("robot1/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base1 action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base1 server")
        
        self.cmd_vel_pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.move_base2 = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base2 action server...")
        self.move_base2.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(self.locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        self.setpose_pub = rospy.Publisher("speak_string",String,latch=True, queue_size=1)
        #rospy.Subscriber('speak_string'
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
#         rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        rospy.loginfo("*** Rog_result")
        rospy.Subscriber('Rog_result', String, self.speech_command)
        rospy.Subscriber('nav_rob1', myPath , self.cb_nav_rob1)
        rospy.Subscriber('nav_rob2', myPath , self.cb_nav_rob2)
        
        '''
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        # Make sure we have the initial pose
        
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,
            # start with a new random sequence
            if i == n_locations:
                i = 0
                sequence = sample(self.locations, n_locations)
                # Skip over first location if it is the same as
                # the last location
                if sequence[0] == last_location:
                    i = 1
            
            # Get the next location in the current sequence
            location = sequence[i]
                        
            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(self.locations[location].position.x - 
                                    self.locations[last_location].position.x, 2) +
                                pow(self.locations[location].position.y - 
                                    self.locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(self.locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(self.locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # Store the last location for distance calculations
            last_location = location
            
            # Increment the counters
            i += 1
            n_goals += 1
        
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = self.locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
            
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            rospy.sleep(self.rest_time)
        '''
    
    def cb_nav_rob1(self,my_path): 
#         print "hello *********"
        robotName = my_path.name;
        print "robot Name: ",robotName
        print "position numbers :",len(my_path.path.poses)
        for PoseStamped in my_path.path.poses:
            print PoseStamped.pose.position
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = PoseStamped.pose
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
           
    def cb_nav_rob2(self,my_path):
        robotName = my_path.name;
        print "robot Name: ",robotName
        print "position numbers :",len(my_path.path.poses)
        for PoseStamped in my_path.path.poses:
            print PoseStamped.pose.position
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = PoseStamped.pose
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base2.send_goal(self.goal)
            finished_within_time = self.move_base2.wait_for_result(rospy.Duration(300)) 
            if not finished_within_time:
                self.move_base2.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base2.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))   
    def speech_command(self,command): 
        #self.command = command   
        print    command.data
        if command.data == "开始导航":
            print "ok"
       
        self.goal = MoveBaseGoal()
        if command.data == "去小会议室":
            self.goal.target_pose.pose = self.locations['small_meeting']
            self.setpose_pub.publish("好的主人 我马上就去小会议室")
        if command.data == "去大会议室":
            self.goal.target_pose.pose = self.locations['big_meeting']
            self.setpose_pub.publish("好的主人 我马上就去大会议室")
        if command.data == "去休息室":
            self.goal.target_pose.pose = self.locations['rest']
            self.setpose_pub.publish("好的主人 我马上就休息室")
        if command.data == "去机房":
            self.goal.target_pose.pose = self.locations['jifang']
            self.setpose_pub.publish("好的主人 我马上就去机房")
        if command.data == "回来":
            self.goal.target_pose.pose = self.locations['comeback']
            self.setpose_pub.publish("好的主人 我马上回来")
        
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        # Let the user know where the robot is going next
        rospy.loginfo("Going to: small_meeting")
        
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)
        
        # Allow 5 minutes to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
        
        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                self.setpose_pub.publish("我已经到了 主人")
#                 n_successes += 1
#                 distance_traveled += distance
                rospy.loginfo("State:" + str(state))
            else:
                self.setpose_pub.publish("没找到您说的地方，让我去其他地方吧")
                rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
                  

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
    

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
