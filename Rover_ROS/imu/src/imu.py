#! /usr/bin/env python
import time
import rospy
import numpy as np

from sensor_msgs.msg import Imu, MagneticField, JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

imu_node = Imu()
mag = MagneticField()

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/yaw', Imu, queue_size=10)
		self.pub1 = rospy.Publisher('/mag_calibrated', MagneticField, queue_size=10)

		#self.cuenta = 0

		self.xmax = 0
		self.ymax = 0
		self.xmin = 0
		self.ymin = 0

                self.x = 0
                self.y = 0
                self.z = 0
                self.yaw = 0
		self.yaw_1 = 0
		self.count = 0

		self.mag_yaw = 0

		#VARIABLE RORATION MATRIX
		self.acc = np.zeros([2,1])
		self.w = 0
		self.RB = np.zeros([2,2])
		self.RB_ = np.zeros([2,1])

		#VARIABLES FILTER IMU_ACC
		self.accx = 0.0
		self.accx_1 = 0.0
		self.accx_2 = 0.0

		self.accy = 0.0
		self.accy_1 = 0.0
		self.accy_2 = 0.0

		self.acc_x = 0.0
		self.acc_x_1 = 0.0
		self.acc_x_2 = 0.0

		self.acc_y = 0.0
		self.acc_y_1 = 0.0
		self.acc_y_2 = 0.0

		#VARIABLES BIAS
		self.free_acc_z = 0.0
		self.free_acc_y = 0.0
		self.free_acc_x = 0.0

		self.mag_z = 0.0
		self.mag_y = 0.0
		self.mag_x = 0.0

                rospy.Subscriber('/imu/mag', Vector3Stamped, self.mag)
		rospy.Subscriber('/imu/data', Imu, self.imu)
		rospy.Subscriber('/filter/free_acceleration', Vector3Stamped, self.free_acc)

		self.tiempo = 0.0

	def free_acc(self, data):
		self.free_acc_x = data.vector.x
		self.free_acc_y = data.vector.y
		self.free_acc_z = data.vector.z

	def mag(self, data):
		x = data.vector.x
		y = data.vector.y
		z = data.vector.z

		if x>self.xmax: self.xmax = x
		if y>self.ymax: self.ymax = y

		if x<self.xmin: self.xmin = x
		if y<self.ymin: self.ymin = y

		xsf = (self.ymax - self.ymin)/(self.xmax - self.xmin)
		ysf = (self.xmax - self.xmin)/(self.ymax - self.ymin)

		xoff = ((self.xmax - self.xmin)/2 - self.xmax)*xsf
		yoff = ((self.ymax - self.ymin)/2 - self.ymax)*ysf

		self.x = xsf*x + xoff
		self.y = ysf*y + yoff
		self.z = z

		self.mag_yaw = np.arctan2(self.y,self.x)

        def imu(self, data):
		orientation_q = data.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		euler = euler_from_quaternion(orientation_list)
		self.yaw = euler[2]

		#ANGULAR VELOCITY
		self.w = data.angular_velocity.z #- 0.02

		if abs(self.w) < 0.05:
			self.w = 0.0

		#FILTER FOR ACCELEROMETERS
		self.accx = self.free_acc_x
		self.accy = self.free_acc_y
		self.acc_x = 0.0009447*self.accx + 0.001889*self.accx_1 + 0.0009447*self.accx_2 + 1.911*self.acc_x_1 - 0.915*self.acc_x_2
		self.acc_y = 0.0009447*self.accy + 0.001889*self.accy_1 + 0.0009447*self.accy_2 + 1.911*self.acc_y_1 - 0.915*self.acc_y_2

		self.acc = np.array([ [self.acc_x],[self.acc_y] ])

		self.RB = np.array([ [np.cos(self.yaw), -np.sin(self.yaw)],
                                        [np.sin(self.yaw), np.cos(self.yaw)]  ])

		self.RB_ = np.dot(self.RB, self.acc)

		if abs(self.RB_[0,0]) < 0.1:
			self.RB_[0,0] = 0.0

		if abs(self.RB_[1,0]) < 0.1:
			self.RB_[1,0] = 0.0

		#UPDATE VARIABLES
		self.accx_2 = self.accx_1
		self.accx_1 = self.accx
		self.acc_x_2 = self.acc_x_1
		self.acc_x_1 = self.acc_x

		self.accy_2 = self.accy_1
		self.accy_1 = self.accy
		self.acc_y_2 = self.acc_y_1
		self.acc_y_1 = self.acc_y


	def main(self):
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():

			#print("acc_x", self.accx)
			#print("free_acc_x", self.acc_x)
			#print("acc_y", self.accy)
			#print("free_acc_y", self.acc_y)

			#print self.RB_[0,0]
			#print self.RB_[1,0]
			#print " "

			#mag.header.stamp = rospy.get_rostime()
			#mag.header.frame_id = "Magnetometro_calibrado"
			#mag.magnetic_field.x = self.x
			#mag.magnetic_field.y = self.y
			#mag.magnetic_field.z = self.z
			#self.pub1.publish(mag)

			imu_node.header.stamp = rospy.get_rostime()
			imu_node.header.frame_id = "NODE IMU ROTATE"
			imu_node.orientation.x = self.yaw
			imu_node.orientation.y = self.mag_yaw
			imu_node.angular_velocity.z = self.w
			imu_node.linear_acceleration.x = self.RB_[0,0]
			imu_node.linear_acceleration.y = self.RB_[1,0]
			self.pub.publish(imu_node)

			#print imu_node
			rate.sleep()


if __name__=='__main__':
	try:
		rospy.init_node("imu_data_filtered")
		print "Nodo YAW Creado"
		cv = ubication()
		cv.main()

	except rospy.ROSInterruptException:
		pass

