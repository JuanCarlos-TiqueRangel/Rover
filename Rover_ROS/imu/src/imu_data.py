#! /usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField, JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

odom = Odometry()

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/yaw', Odometry, queue_size=10)

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

                rospy.Subscriber('/imu/mag', MagneticField, self.mag)
		rospy.Subscriber('/imu/data', Imu, self.imu)
		rospy.Subscriber('/main_node', JointState, self.calibration)
                rospy.spin()

	def imu(self, data):
		orientation_q = data.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		euler = euler_from_quaternion(orientation_list)
		#self.yaw = (euler[2] + self.yaw_1)*(180/np.pi)
		self.yaw = (euler[2])*(180/np.pi)
		#print self.yaw

                odom.pose.pose.position.x = self.yaw
                odom.header.stamp = rospy.get_rostime()
                self.pub.publish(odom)

        def mag(self, data):
		x = data.magnetic_field.x
		y = data.magnetic_field.y
		z = data.magnetic_field.z

		if x>self.xmax: self.xmax = x
		if y>self.ymax: self.ymax = y

		if x<self.xmin: self.xmin = x
		if y<self.ymin: self.ymin = y

		xsf = (self.ymax - self.ymin)/(self.xmax -self.xmin)
		ysf = (self.xmax - self.xmin)/(self.ymax - self.ymin)

		xoff = ((self.xmax - self.xmin)/2 - self.xmax)*xsf
		yoff = ((self.ymax - self.ymin)/2 - self.ymax)*ysf

		self.x = xsf*x + xoff
		self.y = ysf*y + yoff

		#if self.cuenta == 0:
		#	self.yaw_1 = np.arctan2(-y, x)
		#	self.cuenta = self.cuenta + 1

		#if self.calibration == ['Termino']:
		#	self.yaw_1 = np.arctan2(-self.y, self.x)
		#	self.calibration = []
		#	print self.yaw_1

	def calibration(self, data):
		self.calibration = data.name
		if self.calibration == ['Termino']:
			self.yaw_1 = np.arctan2(-self.y, self.x)
			#self.calibration = []
			print self.yaw_1*180/np.pi


if __name__=='__main__':
        try:
                rospy.init_node('Orientation',anonymous=True, disable_signals=True)
                print "Nodo YAW Creado"
                cv = ubication()
        except rospy.ROSInterruptException:
                pass

