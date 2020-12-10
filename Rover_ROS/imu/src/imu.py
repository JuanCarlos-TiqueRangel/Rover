#! /usr/bin/env python
import time
import rospy
import numpy as np

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

imu_node = Imu()

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/yaw', Imu, queue_size=10)

		#self.cuenta = 0

                self.x = 0
                self.y = 0
                self.z = 0
                self.yaw = 0
		self.yaw_1 = 0
		self.count = 0

		#VARIABLE RORATION MATRIX
		self.acc = np.zeros([2,1])

		self.w = 0.0
		self.w_1 = 0.0
		self.w_2 = 0.0
		self.w_f_1 = 0.0
		self.w_f_2 = 0.0
		self.wf = 0.0

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
		rospy.Subscriber('/imu/data', Imu, self.imu)

		self.tiempo = 0.0

        def imu(self, data):
		orientation_q = data.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		euler = euler_from_quaternion(orientation_list)
		self.yaw = euler[2]

		#ANGULAR VELOCITY
		self.wf = data.angular_velocity.z #- 0.02

		# FILTER LOW PASS FOR ANGULAR VELOCITY
		#self.wf = 0.2452*self.w + 0.2452*self.w_1 + 0.5095*self.wf
		#self.wf = 0.0009447*self.w + 0.001889*self.w_1 + 0.0009447*self.w_2 + 1.911*self.w_f_1 - 0.915*self.w_f_2

		if abs(self.wf) < 0.05:
			self.wf = 0.0

		#FILTER FOR ACCELEROMETERS
		self.accx = data.linear_acceleration.x
		self.accy = data.linear_acceleration.y

		if abs(self.accx) < 0.5:
			self.accx = 0.0
		if abs(self.accy) < 0.5:
			self.accy = 0.0

		self.acc_x = 0.0009447*self.accx + 0.001889*self.accx_1 + 0.0009447*self.accx_2 + 1.911*self.acc_x_1 - 0.915*self.acc_x_2
		self.acc_y = 0.0009447*self.accy + 0.001889*self.accy_1 + 0.0009447*self.accy_2 + 1.911*self.acc_y_1 - 0.915*self.acc_y_2

		self.acc = np.array([ [self.acc_x],[self.acc_y] ])
		#self.acc = np.array([ [self.accx],[self.accy]])

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

		self.w_2 = self.w_1
		self.w_1 = self.w
		self.w_f_2 = self.w_f_1
		self.w_f_1 = self.wf


	def main(self):
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():

			imu_node.header.stamp = rospy.get_rostime()
			imu_node.header.frame_id = "NODE IMU ROTATE"
			imu_node.orientation.x = self.yaw
			imu_node.orientation.y = 0.0
			imu_node.angular_velocity.z = self.wf
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

