# Python libs
import math
import numpy as np

# Ros libs
import rospy

# Ros messages
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

class Controller:

	def __init__(self):

		self.gyro_x = 0.0
		self.gyro_y = 0.0
		self.gyro_z = 0.0

		self.accel_x = 0.0
		self.accel_y = 0.0
		self.accel_z = 0.0

		self.ref = np.pi/180

		self.e_sum = 0.0
		self.e = [0.0, 0.0]
		self.y = [0.0, 0.0]
		self.u_pre = 0.0
		self.u = 0.0
		self.diff_u = 0.0

		self.umax = 0.116
		self.umin = -0.116
		self.KP = 26.0
		self.Ki = 7.47
		self.Kd = 20.0

		self.delta1 = 0.0 #todo anpassen an geünschte Datenstruktur

		self.imu_sub = rospy.Subscriber('/testbot/imu', Float32MultiArray, self.imu_callback)

		self.delta1_pub = rospy.Publisher('/testbot/delta1', Float64, queue_size=10)

	def control(self):
		self.diff_u = 0.0

		# insert new error in list and pop oldest value
		self.e.insert(0, self.ref - y[1])
		del self.e[-1]
		self.e_sum += self.e[0]

		#todo calculate dt
		I_anteil = dt * self.e
		D_anteil = (self.e[0] - self.e[1]) / dt
		self.u_pre = self.Kp * self.e[0] + self.Ki * I_anteil + self.Kd * D_anteil

		if u_pre > umax
			self.diff_u = self.umax - self.u_pre

		if u_pre < umin
			self.diff_u = self.umin - self.u_pre

		if self.diff_u != 0
			I_anteil = self.e[0] + 1 / self.Ki * self.diff_u

		#todo weiterimplementieren

	def publish_all(self):
        self.delta1.publish(self.delta1)

	def imu_callback(self, msg):
		self.gyro_x = msg.data[0]       # rad/s
        self.gyro_y = msg.data[1]       
        self.gyro_z = msg.data[2]       

        self.accel_x = msg.data[3]     
        self.accel_y = msg.data[4]      
        self.accel_z = msg.data[5]      

def talker():
    rospy.init_node('/testbot/controller', anonymous=True)
    ctrl = Controller()
    rate = rospy.Rate(100) # in Hz
    while not rospy.is_shutdown():
    	ctrl.control()
        ctrl.publish_all()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass  