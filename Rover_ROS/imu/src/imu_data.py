#! /usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

odom = Odometry()

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/yaw', Odometry, queue_size=10)

                self.x = 0
                self.y = 0
                self.z = 0
                self.yaw = 0
		self.yaw_1 = 0
		self.count = 0

                rospy.Subscriber('/imu/mag', MagneticField, self.mag)
		rospy.Subscriber('/imu/data', Imu, self.imu)
                rospy.spin()

	def imu(self, data):
		orientation_q = data.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		euler = euler_from_quaternion(orientation_list)
		self.yaw_1 = (euler[2] + self.yaw)*(180/np.pi)

                odom.pose.pose.position.x = self.yaw_1
                odom.header.stamp = rospy.get_rostime()
                self.pub.publish(odom)


        def mag(self, data):
		self.x = data.magnetic_field.x
		self.y = data.magnetic_field.y
		self.z = data.magnetic_field.z

		if self.count == 0:
			self.yaw = np.arctan2(-self.y, self.x)
			self.count = self.count+1

                #print self.yaw*(180/np.pi)
if __name__=='__main__':
        try:
                rospy.init_node('Orientation',anonymous=True, disable_signals=True)
                print "Nodo YAW Creado"
                cv = ubication()
        except rospy.ROSInterruptException:
                pass

