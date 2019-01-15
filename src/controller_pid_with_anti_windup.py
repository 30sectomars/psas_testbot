#!/usr/bin/env python2
# Python libs
import math

# Ros libs
import rospy

# Ros messages
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

LOOP_RATE_IN_HZ = 100
G = 9.81

# IMU offset in real world
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
	V_MAX = rospy.get_param('/v_max')
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
		rospy.loginfo(OFFSET_Y)

		self.delta1 = 0.0

		if SIMULATION:
			self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
		else:
			self.imu_sub = rospy.Subscriber('/testbot/imu', Float32MultiArray, self.imu_callback)

		self.delta1_pub = rospy.Publisher('/testbot/delta1', Float64, queue_size=10)

		self.e_pub = rospy.Publisher('/controller/e', Float64, queue_size=10)
		self.y_pub = rospy.Publisher('/controller/y', Float64, queue_size=10)
		self.u_pre_pub = rospy.Publisher('/controller/u_pre', Float64, queue_size=10)
		self.u_pub = rospy.Publisher('/controller/u', Float64, queue_size=10)
		self.diff_u_pub = rospy.Publisher('/controller/diff_u', Float64, queue_size=10)
		self.e_sum_pub = rospy.Publisher('/controller/e_sum', Float64, queue_size=10)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		msg = Twist()
		msg.linear.x = V_MAX
		self.vel_pub.publish(msg)

		rospy.on_shutdown(self.shutdown)

	def control(self):
		self.diff_u = 0.0

		# insert new error in list and pop oldest value
		self.e.insert(0, self.ref - self.y[1])
		del self.e[-1]
		self.e_sum += self.e[0]
		rospy.loginfo("e = %f", -self.e[0])

		I_anteil = self.dt * -self.e_sum
		#I_anteil = 0.0
		D_anteil = (self.e[0] - self.e[1]) / self.dt
		#D_anteil = 0.0
		self.u_pre = self.Kp * self.e[0] + self.Ki * I_anteil + self.Kd * D_anteil

		if self.u_pre > self.umax:
			self.diff_u = self.umax - self.u_pre

		if self.u_pre < self.umin:
			self.diff_u = self.umin - self.u_pre

		if self.diff_u != 0:
			I_anteil = (1.0 / self.Ki) * self.diff_u + self.e[0]

		# 1e-5 equivalent to 10**-5 --> readability
		# self.y.insert(0, ((2 * self.y[1]) - self.y[2] + (2.5 * 1e-5 * (self.u[1] + self.u[2]))))
		# getting some weird values from accel_y --> 44++
		#rospy.loginfo("accel_y = %f", self.accel_y)
		if (self.accel_y/G <= 1.0) & (self.accel_y/G > -1.0):
			self.y.insert(0, math.asin(self.accel_y/G) - OFFSET_Y)
			del self.y[-1]

		self.u.insert(0,self.Kp * self.e[0] + self.Ki * I_anteil + self.Kd * D_anteil)
		del self.u[-1]

		self.delta1 = math.tan(0.015 / 0.05 * self.u[0]) * 180 / math.pi
		#rospy.loginfo("y = %f",self.y[0])
		#rospy.loginfo("delta1 = %f",self.delta1)

	def publish_all(self):
		self.delta1_pub.publish(self.delta1)
		self.e_pub.publish(self.e[0])
		self.y_pub.publish(self.y[0])
		self.u_pre_pub.publish(self.u_pre)
		self.u_pub.publish(self.u[0])
		self.diff_u_pub.publish(self.diff_u)
		self.e_sum_pub.publish(self.e_sum)

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