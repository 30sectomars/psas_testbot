#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 31 09:37:37 2018

@author: nschweizer
"""

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class Euler:
    
    def __init__(self):
        self.sub = rospy.Subscriber('/testbot/imu', Imu, self.euler_callback)
        self.pub_state = rospy.Publisher('/state', Float64, queue_size=10)
        self.pub_setpoint = rospy.Publisher('/setpoint', Float64, queue_size=10)

    def euler_callback (self,msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    
        self.pub_state.publish(roll)
        self.pub_setpoint.publish(0.0)
    

if __name__ == '__main__':
    try:
        rospy.init_node('euler_angles')
        euler = Euler()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException: pass    
