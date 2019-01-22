#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 21 10:42:00 2018

@author: nschweizer
"""

#################### import ##########################
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

############# node initialization
rospy.init_node("teleop")

############# definitions of variables #################
twist = Twist()

############ definitions of functions ##################
def callback(msg):
    twist.linear.x = msg.axes[1] * 0.05
    twist.angular.z = msg.axes[3] * 10
    
#### definition of publisher/subscriber and services ###
rospy.Subscriber('joy', Joy, callback)

pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

################### main program ######################
r = rospy.Rate(25) #25 Hz
while not rospy.is_shutdown():
    pub.publish(twist)
    r.sleep()
    print twist
#rospy.spin()