#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 10 11:35:01 2018

@author: nschweizer
"""

# libraries
import math

# Ros libraries
import rospy

# Ros Messages
from std_msgs.msg import Float64
from std_msgs.msg import Int16MultiArray


############# definitions of constants #################

b_ID1=-0.025754
m_ID1=0.0021497
b_ID2=-0.0045794
m_ID2=0.0020619
b_ID3=-0.039143
m_ID3=0.0021794
b_ID4=-0.00097619
m_ID4=0.0020581
b_ID5=-0.021706
m_ID5=0.0021597
b_ID6=0.020952
m_ID6=0.0020542
b_ID7=-0.034841
m_ID7=0.0021722
b_ID8=0.0051429
m_ID8=0.0020311

bb = [b_ID1, b_ID2, b_ID3, b_ID4, b_ID5, b_ID6, b_ID7, b_ID8]
mm = [m_ID1, m_ID2, m_ID3, m_ID4, m_ID5, m_ID6, m_ID7, m_ID8]

############# definitions of variables #################

speeds = [0] * 8
joints = [0] * 3

############ definitions of functions ##################
def callback_ID1(msg):
    global speeds
    speeds[0] = msg.data
    
def callback_ID2(msg):
    global speeds
    speeds[1] = msg.data
    
def callback_ID3(msg):
    global speeds
    speeds[2] = msg.data
    
def callback_ID4(msg):
    global speeds
    speeds[3] = msg.data
    
def callback_ID5(msg):
    global speeds
    speeds[4] = msg.data

def callback_ID6(msg):
    global speeds
    speeds[5] = msg.data
    
def callback_ID7(msg):
    global speeds
    speeds[6] = msg.data
    
def callback_ID8(msg):
    global speeds
    speeds[7] = msg.data
    
def callback_ID11(msg):
    global joints
    joints[0] = msg.data
    
def callback_ID12(msg):
    global joints
    joints[1] = msg.data
    
def callback_ID13(msg):
    global joints
    joints[2] = msg.data
    
    
#### definition of publisher/subscriber and services ###
def listener():
    rospy.Subscriber('/testbot/wheel1_controller/command', Float64, callback_ID1)
    rospy.Subscriber('/testbot/wheel2_controller/command', Float64, callback_ID2)
    rospy.Subscriber('/testbot/wheel3_controller/command', Float64, callback_ID3)
    rospy.Subscriber('/testbot/wheel4_controller/command', Float64, callback_ID4)
    rospy.Subscriber('/testbot/wheel5_controller/command', Float64, callback_ID5)
    rospy.Subscriber('/testbot/wheel6_controller/command', Float64, callback_ID6)
    rospy.Subscriber('/testbot/wheel7_controller/command', Float64, callback_ID7)
    rospy.Subscriber('/testbot/wheel8_controller/command', Float64, callback_ID8)
    
    rospy.Subscriber('/testbot/joint1_position_controller/command', Float64, callback_ID11)
    rospy.Subscriber('/testbot/joint2_position_controller/command', Float64, callback_ID12)
    rospy.Subscriber('/testbot/joint3_position_controller/command', Float64, callback_ID13)

def talker():
    wheels_pub = rospy.Publisher('wheel_vel', Int16MultiArray, queue_size=10)
    joints_pub = rospy.Publisher('joint_ang', Int16MultiArray, queue_size=10)
    rate = rospy.Rate(20)
    wheels = [0] * 8
    ww = Int16MultiArray()
    angles = [0] * 3
    aa = Int16MultiArray()
    while not rospy.is_shutdown():
        for i in range(len(wheels)):
            if speeds[i] < 0.01:
                wheels[i] = 0
            else:
                speed = (speeds[i]/(2*math.pi)-bb[i])/mm[i]
                wheels[i] = round(speed,0)
        ww.data = wheels
        for i in range(len(angles)):
            joint = 513 + 3.41*joints[i]*180/math.pi
            angles[i] = round(joint,0)
        aa.data = angles
        wheels_pub.publish(ww)
        joints_pub.publish(aa)
        rate.sleep()
    
################### main program ######################
if __name__ == '__main__':
    try:
        rospy.init_node('testbot', anonymous=True)
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass