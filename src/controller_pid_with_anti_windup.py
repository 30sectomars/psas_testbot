#!/usr/bin/env python2
# Python libs
import math

# Ros libs
import rospy

# Ros messages
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

LOOP_RATE_IN_HZ = 100
G = 9.81

if rospy.has_param('/use_simulation'):
	SIMULATION = rospy.get_param('/use_simulation')
else:
	SIMULATION = False

class Controller:

	def __init__(self):

		self.gyro_x = 0.0
		self.gyro_y = 0.0
		self.gyro_z = 0.0

		self.accel_x = 0.0
		self.accel_y = 0.0
		self.accel_z = 0.0

		self.ref = 0.0

		self.e_sum = 0.0
		self.e = [0.0, 0.0]
		self.y = [0.0, 0.0, 0.0]
		self.u_pre = 0.0
		self.u = [0.0, 0.0, 0.0]
		self.diff_u = 0.0

		self.umax = 0.116
		self.umin = -0.116
		self.Kp = 26.0
		self.Ki = 7.47
		self.Kd = 20.0

		self.dt = 1.0 / LOOP_RATE_IN_HZ
		rospy.loginfo("dt = %f", self.dt)
		rospy.loginfo(SIMULATION)

		self.delta1 = 0.0

		if SIMULATION:
			self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
		else:
			self.imu_sub = rospy.Subscriber('/testbot/imu', Float32MultiArray, self.imu_callback)

		self.delta1_pub = rospy.Publisher('/testbot/delta1', Float64, queue_size=10)

	def control(self):
		self.diff_u = 0.0

		# insert new error in list and pop oldest value
		self.e.insert(0, self.ref - self.y[1])
		del self.e[-1]
		self.e_sum += self.e[0]
		#rospy.loginfo("e_sum = %f", self.e_sum)

		I_anteil = self.dt * self.e_sum
		D_anteil = (self.e[0] - self.e[1]) / self.dt
		self.u_pre = self.Kp * self.e[0] + self.Ki * I_anteil + self.Kd * D_anteil

		if self.u_pre > self.umax:
			self.diff_u = self.umax - self.u_pre

		if self.u_pre < self.umin:
			self.diff_u = self.umin - self.u_pre

		if self.diff_u != 0:
			I_anteil = 1 / self.Ki * self.diff_u + self.e[0]

		# 1e-5 equivalent to 10**-5 --> readability
		# self.y.insert(0, ((2 * self.y[1]) - self.y[2] + (2.5 * 1e-5 * (self.u[1] + self.u[2]))))
		# getting some weird values from accel_y --> 44++
		#rospy.loginfo("accel_y = %f", self.accel_y)
		if (self.accel_y/G <= 1.0) & (self.accel_y/G > -1.0):
			self.y.insert(0, math.asin(self.accel_y/G))
			del self.y[-1]

		self.u.insert(0,self.Kp * self.e[0] + self.Ki * I_anteil + self.Kd * D_anteil)
		del self.u[-1]

		self.delta1 = -0.015 * math.tan(self.u[0]) * 180 / math.pi

		#rospy.loginfo(self.y)
		#rospy.loginfo(self.u[0])

	def publish_all(self):
		self.delta1_pub.publish(self.delta1)

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

		#self.y.insert(0,math.asin(self.accel_y))
		#del self.y[-1]
		#rospy.loginfo(self.y)   

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