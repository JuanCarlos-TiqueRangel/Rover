#! /usr/bin/env python
import serial
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/Position', Odometry, queue_size=10)

		self.enc_ts = 0.05

		self.W_Left = 0.0
		self.W_Right =0.0
		self.W_Left_1 = 0.0
		self.W_Right_1 = 0.0

		self.WL = 0.0
		self.WR = 0.0
		self.WL_1 = 0.0
		self.WR_1 = 0.0

		self.Left = 0.0
		self.Left_1 = 0.0
		self.Right = 0.0
		self.Right_1 = 0.0

		self.PL_1 = 0.0
		self.PR_1 = 0.0

		self.counter = 0.0
		self.counter_1 = 0.0
		self.deltatiempo = 0.0
		self.deltatiempo_1 = 0.0

		rospy.Subscriber('/enc_data', JointState, self.read_enc)
		#self.rate = rospy.Rate(10)
                rospy.spin()

        def read_enc(self,data):
		odom = Odometry()
		self.counter = data.velocity[2]
		self.deltatiempo = self.counter - self.counter_1

		if self.deltatiempo == 0:
			self.deltatiempo = self.deltatiempo_1

		# Angular Velocity rad/s and Lineal Velocity m/s
		# 1500 son pulsos por RPM que tiene el encoder

		self.W_Left = (data.velocity[0]-self.PL_1)*(2*np.pi)/(1500*self.enc_ts*self.deltatiempo)
		self.W_Right = (data.velocity[1]-self.PR_1)*(2*np.pi)/(1500*self.enc_ts*self.deltatiempo)

		#saturar la aceleracion
		if abs(self.W_Left - self.W_Left_1) > 20.0 or abs(self.W_Right - self.W_Right_1) > 20.0:
			self.W_Left = self.W_Left_1
			self.W_Right = self.W_Right_1

		#filtro buttter
		self.Left = self.W_Left
		self.Right = self.W_Right
		self.WL = 0.2452*self.Left + 0.2452*self.Left_1 + 0.5095*self.WL
		self.WR = 0.2452*self.Right + 0.2452*self.Right_1 + 0.5095*self.WR

		#publish data
		odom.twist.twist.angular.x = self.WL # Velocidad angular motor izquierdo
		odom.twist.twist.angular.y = self.WR # Velocidad angular motor derecho

		#odom.twist.twist.linear.x = vel_left # velocidad sin filtro
		#odom.twist.twist.linear.y = vel_right # velocidad sin filtro

		odom.header.stamp = rospy.get_rostime()
		self.pub.publish(odom)


		#update variables
		self.counter_1 = self.counter
		self.deltatiempo_1 = self.deltatiempo

		self.PL_1 =  data.velocity[0]
		self.PR_1 = data.velocity[1]

		self.Left_1 = self.Left
		self.Right_1 = self.Right

		self.W_Left_1 = self.W_Left
		self.W_Right_1 = self.W_Right

		#print str(self.W_left) +"\t"+ str(self.W_left*9.5493)
		#print "\t"
		#print str(self.W_Right) + "\t" + str(self.W_Right_1) + '\t' + str(data.pose.covariance[6])
		#print self.WL

if __name__=='__main__':
        try:
                rospy.init_node('Position',anonymous=True, disable_signals=True)
		print "Nodo POS Creado"
                cv = ubication()
        except rospy.ROSInterruptException:
                pass
