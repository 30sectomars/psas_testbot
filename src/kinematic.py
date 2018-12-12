#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#"""
#Created on Thu Jul 26 10:39:52 2018
#
#@author: nschweizer
#"""

# Python libs
import math

# Ros libraries
import rospy

# Ros Messages
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

scaling = rospy.get_param('scale', 3)

class Robot:
        
    # Link length
    L11 = scaling * 0.015
    L12 = scaling * 0.144
    L21 = scaling * 0.128
    L22 = scaling * 0.043
    L31 = scaling * 0.026
    L32 = scaling * 0.138
    L41 = scaling * 0.144
    L42 = scaling * 0.015
    
    wheel_radius = scaling * 0.02
    wheel_distance = scaling * 0.059
    
 #   v1 = 0.05/0.015 * wheel_radius
    v1 = scaling * 0.05
    delta1 = 0.0 * math.pi / 180 
    
    def __init__(self):
        self.theta1 = 0.0
        self.theta1_d = 0.0
        self.phi2 = 0.0
        self.phi2_d = 0.0
        self.v2 = 0.0
        self.theta2 = 0.0
        self.theta2_d = 0.0
        self.phi3 = 0.0
        self.phi3_d = 0.0
        self.v3 = 0.0
        self.theta3 = 0.0
        self.theta3_d = 0.0
        self.phi4 = 0.0
        self.phi4_d = 0.0
        self.v4 = 0.0
        
        self.omega1 = 0.0
        self.omega2 = 0.0
        self.omega3 = 0.0
        self.omega4 = 0.0
        self.omega5 = 0.0
        self.omega6 = 0.0
        self.omega7 = 0.0
        self.omega8 = 0.0
        
        self.time_old = rospy.get_rostime()
        
        # ROS publisher
        self.joint1_pub = rospy.Publisher('/testbot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/testbot/joint2_position_controller/command', Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher('/testbot/joint3_position_controller/command', Float64, queue_size=10)
        
        self.wheel1_pub = rospy.Publisher('/testbot/wheel1_controller/command', Float64, queue_size=10)
        self.wheel2_pub = rospy.Publisher('/testbot/wheel2_controller/command', Float64, queue_size=10)
        self.wheel3_pub = rospy.Publisher('/testbot/wheel3_controller/command', Float64, queue_size=10)
        self.wheel4_pub = rospy.Publisher('/testbot/wheel4_controller/command', Float64, queue_size=10)
        self.wheel5_pub = rospy.Publisher('/testbot/wheel5_controller/command', Float64, queue_size=10)
        self.wheel6_pub = rospy.Publisher('/testbot/wheel6_controller/command', Float64, queue_size=10)
        self.wheel7_pub = rospy.Publisher('/testbot/wheel7_controller/command', Float64, queue_size=10)
        self.wheel8_pub = rospy.Publisher('/testbot/wheel8_controller/command', Float64, queue_size=10)
        
        # ROS subscriber
        self.delta1_sub = rospy.Subscriber('/testbot/delta1', Float64, self.delta1_callback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
        
        
    def kinematic(self):
        
        time_new = rospy.get_rostime()
        dt = (time_new - self.time_old).to_sec()
        self.time_old = time_new
        
        # Module 1
        self.theta1 = self.theta1 + self.theta1_d * dt
        self.theta1_d = math.tan(self.delta1) * self.v1 / self.L11
        
        beta1 = math.atan(-self.L12/self.L11 * math.tan(self.delta1))
        
        # Module 2
        self.phi2 = self.phi2 + self.phi2_d * dt
        self.phi2_d = -self.v1 * (math.sin(self.phi2)/self.L21 + math.tan(self.delta1)/self.L11 * (self.L12/self.L21 * math.cos(self.phi2) + 1))
        
        delta2 = beta1 - self.phi2
        beta2 = math.atan(-self.L22/self.L21 * math.tan(delta2))
        self.v2 = self.v1 * math.cos(delta2)/math.cos(beta2)
        
        self.theta2 = self.theta2 + self.theta2_d * dt
        self.theta2_d = math.tan(delta2) * self.v2 / self.L21
        
        # Module 3
        self.phi3 = self.phi3 + self.phi3_d * dt
        self.phi3_d = -self.v2 * (math.sin(self.phi3)/self.L31 + math.tan(delta2)/self.L21 * (self.L22/self.L31 * math.cos(self.phi3) + 1))
        
        delta3 = beta2 - self.phi3
        beta3 = math.atan(-self.L32/self.L31 * math.tan(delta3))
        self.v3 = self.v2 * math.cos(delta3)/math.cos(beta3)
        
        self.theta3 = self.theta3 + self.theta3_d * dt
        self.theta3_d = math.tan(delta3) * self.v3 / self.L31
        
        # Module 3
        self.phi4 = self.phi4 + self.phi4_d * dt
        self.phi4_d = -self.v3 * (math.sin(self.phi4)/self.L41 + math.tan(delta3)/self.L31 * (self.L32/self.L41 * math.cos(self.phi4) + 1))
        
        delta4 = beta3 - self.phi4
        beta4 = math.atan(-self.L42/self.L41 * math.tan(delta4))
        self.v4 = self.v3 * math.cos(delta4)/math.cos(beta4)
        
#        self.theta4 = self.theta4 + self.theta4_d * dt
#        self.theta4_d = math.tan(self.delta4) * self.v4 / self.L41
        
        self.omega1 = self.v1 * (1 - self.wheel_distance / 2 * math.tan(self.delta1) / self.L11) / self.wheel_radius
        self.omega2 = self.v1 * (1 + self.wheel_distance / 2 * math.tan(self.delta1) / self.L11) / self.wheel_radius
        self.omega3 = self.v2 * (1 - self.wheel_distance / 2 * math.tan(delta2) / self.L21) / self.wheel_radius
        self.omega4 = self.v2 * (1 + self.wheel_distance / 2 * math.tan(delta2) / self.L21) / self.wheel_radius
        self.omega5 = self.v3 * (1 - self.wheel_distance / 2 * math.tan(delta3) / self.L31) / self.wheel_radius
        self.omega6 = self.v3 * (1 + self.wheel_distance / 2 * math.tan(delta3) / self.L31) / self.wheel_radius
        self.omega7 = self.v4 * (1 - self.wheel_distance / 2 * math.tan(delta4) / self.L41) / self.wheel_radius
        self.omega8 = self.v4 * (1 + self.wheel_distance / 2 * math.tan(delta4) / self.L41) / self.wheel_radius

        
    def publish_all(self):
        self.joint1_pub.publish(self.phi2)
        self.joint2_pub.publish(self.phi3)
        self.joint3_pub.publish(self.phi4)
        self.wheel1_pub.publish(self.omega1)
        self.wheel2_pub.publish(self.omega2)
        self.wheel3_pub.publish(self.omega3)
        self.wheel4_pub.publish(self.omega4)
        self.wheel5_pub.publish(self.omega5)
        self.wheel6_pub.publish(self.omega6)
        self.wheel7_pub.publish(self.omega7)
        self.wheel8_pub.publish(self.omega8)
        
    
    def delta1_callback(self, msg):
        if msg.data > 2.0:
            self.delta1 = 2.0
        elif msg.data < -2.0:
            self.delta1 = 2.0
        else:
            self.delta1 = msg.data
        self.delta1 = self.delta1 / 180 * math.pi
        
    
    def twist_callback(self, msg):
        self.v1 = msg.linear.x * 0.05 * scaling
        self.delta1 = msg.angular.z * 2 * math.pi / 180
        print self.delta1
        


def talker():
    rospy.init_node('kinematic', anonymous=True)
    testbot = Robot()
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        testbot.kinematic()
        testbot.publish_all()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass    
