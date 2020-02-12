#! /usr/bin/env python
import serial
import rospy
import message_filters
import numpy as np

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class adq_datos(object):
        def __init__(self):
                self.datos = [0,0,0,0,0,0]

                rospy.Subscriber('/yaw', Odometry, self.synchronize_imu)
                rospy.Subscriber('/PWM', JointState, self.synchronize_pwm)
                rospy.Subscriber('/Position', Odometry, self.synchronize_pos)
                rospy.Subscriber('/gps_dis', Odometry, self.synchronize_gps)

        def synchronize_imu(self, yaw):
                self.datos[0] = yaw.pose.pose.position.x

	def synchronize_pwm(self, pwm):
                self.datos[1] = pwm.velocity[0]
                self.datos[2] = pwm.velocity[1]

	def synchronize_pos(self, encoder):
                self.datos[3] = encoder.twist.twist.angular.x
                self.datos[4] = encoder.twist.twist.angular.y

	def synchronize_gps(self, gps):
                self.datos[5] = gps.pose.pose.position.x
		#self.tiempo = rospy.get_rostime()

	def w_data(self):
		rate = rospy.Rate(10) # publish at 10hz
		while not rospy.is_shutdown():
			tiempo = rospy.get_rostime()
                	f = open('file.txt','a')

                	f.write(format(self.datos[0])+'\t'+format(self.datos[1])+
			'\t'+format(self.datos[2])+'\t'+format(self.datos[3])+
			'\t'+format(self.datos[4])+'\t'+format(self.datos[5])+
			'\t'+format(tiempo.nsecs/1000000)+'\n')
                	f.close()
			print tiempo.nsecs
			rate.sleep()

if __name__=='__main__':
        try:
                rospy.init_node('save_data',anonymous=True, disable_signals=True)
                print "Nodo Creado"
                cv = adq_datos()
		cv.w_data()
        except rospy.ROSInterruptException:
                pass

