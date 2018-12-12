#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  2 08:20:41 2018

@author: nschweizer
"""

# Python libs
import math
import numpy as np
from quaternion import quaternion

# Ros libraries
import rospy

# Ros Messages
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray


class Filter:
    
    def __init__(self):
        
        self.sub = rospy.Subscriber('/testbot/imu', Float32MultiArray, self.imu_callback)
        
        self.pub_roll = rospy.Publisher('/roll', Float64, queue_size=10)
        self.pub_pitch = rospy.Publisher('/pitch', Float64, queue_size=10)
        self.pub_yaw = rospy.Publisher('/yaw', Float64, queue_size=10)
        self.pub_setpoint = rospy.Publisher('/setpoint', Float64, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu', Imu, queue_size=10)
        self.pub_delta = rospy.Publisher('/testbot/delta1', Float64, queue_size=10)
        
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        
        self.q = quaternion(1, 0, 0, 0)
        
        self.gamma_accel = 0.002    # Static Weighting Accelerometer
        
        self.time_old = rospy.get_rostime()
        
        
    def imu_callback(self, msg):
        self.gyro_x = msg.data[0]       # rad/s
        self.gyro_y = msg.data[1]       # Achsen: -
        self.gyro_z = msg.data[2]       # Achsen: -

        self.accel_x = msg.data[3]      # m/s²
        self.accel_y = msg.data[4]      # Achsen: -
        self.accel_z = msg.data[5]      # Achsen: -
        
        w_x = self.gyro_x
        w_y = self.gyro_y
        w_z = self.gyro_z
        
        time_new = rospy.get_rostime()
        dt = (time_new - self.time_old).to_sec()
        self.time_old = time_new
        
        self.integration_gyro(dt,w_x,w_y,w_z)
        self.accelerometer()
        (roll, pitch, yaw) = self.euler_from_quat()
        
        self.pub_roll.publish(roll)
        self.pub_pitch.publish(pitch)
        self.pub_yaw.publish(yaw)
        self.pub_setpoint.publish(0.0)
        
        imu = Imu()
        imu.header.frame_id = "imu_frame"
        imu.orientation.x = self.q.x
        imu.orientation.y = self.q.y
        imu.orientation.z = self.q.z
        imu.orientation.w = self.q.w
        imu.angular_velocity.x = self.gyro_x
        imu.angular_velocity.y = self.gyro_y
        imu.angular_velocity.z = self.gyro_z
        imu.linear_acceleration.x = self.accel_x
        imu.linear_acceleration.y = self.accel_y
        imu.linear_acceleration.z = self.accel_z
        
        self.pub_imu.publish(imu)

        rospy.loginfo(np.arcsin(self.accel_y/9.81))
        
#        delta = -5.0*roll - 0.0*math.sin(pitch)/roll
        
#        self.pub_delta.publish(delta)
        
        
    def integration_gyro(self, dt, w_x, w_y, w_z):
        
        omega_norm = math.sqrt(w_x*w_x + w_y*w_y + w_z*w_z)
        
        beta = math.sin(omega_norm*dt/2.0)
        alpha = math.cos(omega_norm*dt/2.0)
        
        # SAFE DIVISION
        if omega_norm >= 0.001:
            beta = beta / omega_norm
        else:
            beta = dt / 2.0
            
        # Hilfsgrößen
        beta1 = beta * w_x
        beta2 = beta * w_y
        beta3 = beta * w_z
        
        # Eigentlicher Integrationsschritt
        self.q.w = alpha*self.q.w - beta1*self.q.x - beta2*self.q.y - beta3*self.q.z
        self.q.x = beta1*self.q.w + alpha*self.q.x + beta3*self.q.y - beta2*self.q.z
        self.q.y = beta2*self.q.w - beta3*self.q.x + alpha*self.q.y + beta1*self.q.z
        self.q.z = beta3*self.q.w + beta2*self.q.x - beta1*self.q.y + alpha*self.q.z
        
        # Normalisieren
        q_norm_inv = 1.0 / (math.sqrt(self.q.w*self.q.w + self.q.x*self.q.x + self.q.y*self.q.y + self.q.z*self.q.z))
        self.q = self.q * q_norm_inv
        
        # Check to
        if self.q.w<0:
            self.q = -self.q
            
            
    def accelerometer(self):
        
        # Norm of external Acceleration
        acc_norm = math.sqrt(self.accel_x*self.accel_x + self.accel_y*self.accel_y + self.accel_z*self.accel_z)
        
        # Dynamic correction
        gamma_accel_cor = 1.0
        
        # Nur berechnen, wenn
        if acc_norm>=8:
            
            # Richtung der Beschleuingung - Annahme: acc=-g
            acc_normalized_x = -self.accel_x / acc_norm
            acc_normalized_y = -self.accel_y / acc_norm
            acc_normalized_z = -self.accel_z / acc_norm
            
            # Calculate procentual magnitude error -> external acceleration?
            magnitude_error_acc = math.fabs(acc_norm-9.81)
            magnitude_error_acc = magnitude_error_acc / 9.81
            
            # Check for external acceleration and calc dynamic alpha modification
            if magnitude_error_acc<=0.1:
                gamma_accel_cor = 1.0
            elif magnitude_error_acc>=0.2:
                gamma_accel_cor = 0.0
            else:
                gamma_accel_cor = 2.0 - 10.0*magnitude_error_acc
                
            
            gamma_accel_cor = gamma_accel_cor*self.gamma_accel
            
            # Expected acceleration due to current attitude estimate
            g_exp_x = 2.0*(self.q.x * self.q.z - self.q.w * self.q.y)
            g_exp_y = 2.0*(self.q.y * self.q.z + self.q.w * self.q.x)
            g_exp_z = self.q.w*self.q.w - self.q.x*self.q.x - self.q.y*self.q.y + self.q.z*self.q.z
            
            # Hilfsgrößen -> middle between g_exp and g_meas
            v_mid_x = g_exp_x + acc_normalized_x
            v_mid_y = g_exp_y + acc_normalized_y
            v_mid_z = g_exp_z + acc_normalized_z
            
            # Normalisieren
            v_mid_Inorm = 1.0 / (v_mid_x*v_mid_x + v_mid_y*v_mid_y + v_mid_z*v_mid_z)
            
            if not (math.isnan(v_mid_Inorm) or math.isinf(v_mid_Inorm)):
                
                # Normalize middle vector
                v_mid_x *= v_mid_Inorm
                v_mid_y *= v_mid_Inorm
                v_mid_z *= v_mid_Inorm
                
                q_delta_accel = np.quaternion(1, 0, 0, 0)
                
                q_delta_accel.w = g_exp_x*v_mid_x + g_exp_y*v_mid_y + g_exp_z*v_mid_z
                q_delta_accel.x = -(g_exp_y*v_mid_z - v_mid_y*g_exp_z)
                q_delta_accel.y = -(g_exp_z*v_mid_x - v_mid_z*g_exp_x)
                q_delta_accel.z = -(g_exp_x*v_mid_y - v_mid_x*g_exp_y)
                
                q_final_accel = np.quaternion(1, 0, 0, 0)
                
                q_final_accel.w = self.q.w*q_delta_accel.w - self.q.x*q_delta_accel.x - self.q.y*q_delta_accel.y - self.q.z*q_delta_accel.z
                q_final_accel.x = self.q.w*q_delta_accel.x + self.q.x*q_delta_accel.w + self.q.y*q_delta_accel.z - self.q.z*q_delta_accel.y
                q_final_accel.y = self.q.w*q_delta_accel.y - self.q.x*q_delta_accel.z + self.q.y*q_delta_accel.w + self.q.z*q_delta_accel.x
                q_final_accel.z = self.q.w*q_delta_accel.z + self.q.x*q_delta_accel.y - self.q.y*q_delta_accel.x + self.q.z*q_delta_accel.w
                
                
                
                # Protect from Unwinding
                if q_final_accel.w<0:
                    q_final_accel = -q_final_accel
                    
                if math.fabs(q_delta_accel.w)>0.95:
                    # LERP
                    self.q = self.q*(1.0-gamma_accel_cor) + q_final_accel*gamma_accel_cor
                else:
                    # SLERP
                    beta = math.acos(q_delta_accel.w)
                    beta1 = 1.0 / beta
                    
                    beta2 = beta1*math.sin(beta*(1.0-gamma_accel_cor))
                    beta3 = beta1+math.sin(beta*gamma_accel_cor)
                    
                    self.q = self.q*beta2 + q_final_accel*beta3
                    
                # Re-Normailze Quaternion
                q_norm_inv = 1.0 / math.sqrt(self.q.w*self.q.w + self.q.x*self.q.x + self.q.y*self.q.y + self.q.z*self.q.z)
                self.q = self.q*q_norm_inv
                
                # Protect from Unwinding
                if self.q.w<0:
                    self.q = -self.q
            
        else:
            # Magnitude of Acceleration not in the correct range for attitude correction
            # Setzen damit BIAS-Schätzung nicht beeinflusst wird.
            # Sensorfehler oder freier Fall
            q_delta_accel = np.quaternion(1, 0, 0, 0)
            q_final_accel = self.q
            gamma_accel_cor = 0
            
            
    def euler_from_quat(self):
        roll = math.atan2(2.0*self.q.y*self.q.z + 2.0*self.q.w*self.q.x , self.q.z*self.q.z - self.q.y*self.q.y - self.q.x*self.q.x + self.q.w*self.q.w)
        pitch = -math.asin(2.0*self.q.x*self.q.z - 2.0*self.q.w*self.q.y)
        yaw = math.atan2(2.0*self.q.x*self.q.y + 2.0*self.q.w*self.q.z , self.q.x*self.q.x + self.q.w*self.q.w - self.q.z*self.q.z - self.q.y*self.q.y)
            
        return (roll, pitch, yaw)
    
    
        
if __name__ == '__main__':
    try:
        rospy.init_node('complementary_filter')
        filter = Filter()
        rospy.spin()
    except rospy.ROSInterruptException: pass  
        