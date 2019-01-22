#!/usr/bin/env python2
# Python libs
import math
import numpy as np

# Ros libs
import rospy

# Ros messages
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

#Gravity
G = 9.81
FILTER_SIZE = 1
FILTER_SIZE_U = 1

if rospy.has_param('/use_simulation'):
	SIMULATION = rospy.get_param('/use_simulation')
	if SIMULATION:
		OFFSET_Y = 0.0
	else:
		OFFSET_Y = 0.135
else:
	SIMULATION = False
	OFFSET_Y = 0.135

# get v_max
if rospy.has_param('/v_max'):
	V_MAX = 0.5 * rospy.get_param('/v_max')
else:
	V_MAX = 0.05

# get loop rate in hz
if rospy.has_param('/loop_rate_in_hz'):
	LOOP_RATE_IN_HZ = rospy.get_param('/loop_rate_in_hz')
else:
	LOOP_RATE_IN_HZ = 100

class Controller:

	def __init__(self):

		self.gyro_x = 0.0
		self.gyro_y = 0.0
		self.gyro_z = 0.0

		self.accel_x = 0.0
		self.accel_y = 0.0
		self.accel_z = 0.0

		#Ricatti
		#self.k1 = 0.2752
		#self.k2 = 0.0707

		#Ricatti schneller
		self.k1 = 0.4279
		self.k2 = 0.1581

		#Zustandsregler
		#self.k1 = 0.4
		#self.k2 = 0.899

		#Zustandsregler
		#self.k1 = 4.2
		#self.k2 = 2.889

		self.alpha_list = [0.0] * FILTER_SIZE

		#system
		self.Ad = np.matrix([[1.0000, 0.0050],[0.0000, 1.0000]])
		self.Bd = np.matrix([[0.0000],[0.0100]])
		self.Cd = np.matrix([1.0, 1.0])
		#self.R = 0.0256
		self.R = 0.0016
		if SIMULATION:
			self.R = 0.0016
		self.Q = np.matrix([[10.00, 0.00],[0.00, 1.00]])

		self.K = np.matrix([[0.0],[0.0]])

		self.alpha_correction = 0.0
		self.psi_correction = 0.0
		self.p_correction = 0.0

		self.u = [0.0] * FILTER_SIZE_U
		self.u_avg = 0.0
		self.delta1 = 0.0

		if SIMULATION:
			self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
		else:
			self.imu_sub = rospy.Subscriber('/testbot/imu', Float32MultiArray, self.imu_callback)

		self.delta1_pub = rospy.Publisher('/testbot/delta1', Float64, queue_size=10)
		self.u_pub = rospy.Publisher('/controller/u', Float64, queue_size=10)
		self.alpha_correction_pub = rospy.Publisher('/controller/alpha_correction', Float64, queue_size=10)
		self.psi_correction_pub = rospy.Publisher('/controller/psi_correction', Float64, queue_size=10)
		self.p_correction_pub = rospy.Publisher('/controller/p_correction', Float64, queue_size=10)
		self.alpha_pub = rospy.Publisher('/controller/alpha', Float64, queue_size=10)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		msg = Twist()
		msg.linear.x = V_MAX
		self.vel_pub.publish(msg)

		rospy.on_shutdown(self.shutdown)

	def control(self):
		if (self.accel_y/G <= 1.0) & (self.accel_y/G > -1.0):
			self.alpha_list.insert(0, math.asin(self.accel_y/G) - OFFSET_Y)
			del self.alpha_list[-1]

		alpha = sum(self.alpha_list)/len(self.alpha_list)

		# prediction
		alpha_prediction = self.alpha_correction + 0.005 * self.psi_correction 
		psi_prediction = self.psi_correction + 0.01 * self.u_avg
		p_prediction = self.Ad * self.p_correction * self.Ad.T + self.Q
		
		# correction
		self.K = p_prediction * self.Cd.T * np.linalg.inv(self.Cd * p_prediction * self.Cd.T + self.R)

 		self.alpha_correction = alpha_prediction + self.K[0][0] * (alpha - alpha_prediction)
 		self.psi_correction = psi_prediction + self.K[1][0] * (alpha - alpha_prediction)
 		self.p_correction = (np.identity(2) - self.K * self.Cd) * p_prediction

 		# Zustandsregler langsam
		self.u.insert(0, -self.k1 * self.alpha_correction - self.k2 * self.psi_correction)
		del self.u[-1]
		self.u_avg = sum(self.u)/len(self.u)
		self.delta1 = -math.tan(0.015 / V_MAX * self.u_avg) * 180 / math.pi
		if SIMULATION:
			self.delta1 = -self.delta1
		#rospy.loginfo("u = %f", self.u)
		rospy.loginfo("alpha = %f && delta = %f && u = %f", alpha, self.delta1, self.u_avg)
		#rospy.loginfo("delta = %f", self.delta1)

	def publish_all(self):
		#self.delta1_pub.publish(self.delta1)
		self.u_pub.publish(self.u[0])
		self.alpha_correction_pub.publish(self.alpha_correction)
		self.psi_correction_pub.publish(self.psi_correction)
		self.p_correction_pub.publish(self.p_correction)
		self.alpha_pub.publish(self.alpha_list[0])
		msg = Twist()
		msg.linear.x = V_MAX
		msg.angular.z = self.delta1
		self.vel_pub.publish(msg)

	def imu_callback(self, msg):
		if SIMULATION:
			self.gyro_x = msg.angular_velocity.x
			self.gyro_y = -msg.angular_velocity.y
			self.gyro_z = -msg.angular_velocity.z
			self.accel_x = msg.linear_acceleration.x
			self.accel_y = -msg.linear_acceleration.y
			self.accel_z = -msg.linear_acceleration.z
			#rospy.loginfo("lin_accel_x = %f", self.accel_x)
			#rospy.loginfo("lin_accel_y = %f", self.accel_y)
			#rospy.loginfo("lin_accel_z = %f", self.accel_z)
		else:
			self.gyro_x = msg.data[0]
			self.gyro_y = msg.data[1]
			self.gyro_z = msg.data[2]
			self.accel_x = msg.data[3]
			self.accel_y = msg.data[4]
			self.accel_z = msg.data[5]

	def shutdown(self):
		msg = Twist()
		msg.linear.x = 0.0
		msg.angular.z = 0.0
		self.vel_pub.publish(msg)
		#rospy.loginfo("Controller is shut down")

def talker():
	rospy.init_node('controller', anonymous=True)
	ctrl = Controller()
	rate = rospy.Rate(LOOP_RATE_IN_HZ)
	while not rospy.is_shutdown():
		ctrl.control()
		ctrl.publish_all()
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass