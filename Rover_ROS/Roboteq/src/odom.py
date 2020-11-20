#! /usr/bin/env python
import serial
import rospy
import numpy as np

from math import pi, cos, sin
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

odom = Odometry()
msg = Vector3Stamped()

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)

		self.enc_ts = 0.05

		self.PL = 0.0
		self.PL_1 = 0.0

		self.PR_1 = 0.0
		self.PR = 0.0

		self.delta_SL = 0.0
		self.delta_SR = 0.0

		self.PX = 0.0
		self.PY = 0.0
		self.delta_PX = 0.0
		self.delta_PY = 0.0

		self.theta = 0.0
		self.delta_theta = 0.0

		self.yg = 0.38
		self.delta_s = 0.0
		self.base_width = 0.6

		rospy.Subscriber('/enc_data', JointState, self.read_enc)
		rospy.Subscriber('/kalman_filter', Odometry, self.KF_correct)
		rospy.Subscriber('/yaw', Imu, self.imu)

		self.aux = []
		self.aux2 = []
		self.aux3 = []
		self.aux4 = []
		self.samples = 0

		#VARIABLES VELOCIDAD ANGULAR
		self.W_Left = 0.0
		self.W_Left_1 = 0.0
		self.W_Right = 0.0
		self.W_Right_1 = 0.0

		self.WL = 0.0
		self.WR = 0.0

		self.Left = 0.0
		self.Left_1 = 0.0
		self.Right = 0.0
		self.Right_1 = 0.0

		self.counter = 0.0
		self.counter_1 = 0.0
		self.deltatiempo = 0.0
		self.deltatiempo_1 = 0.0

		#VARIABLES PARA HALLAR LAS COORDENADAS X,Y
		self.Ds = 0.0
		self.Dtheta = 0.0
		self.theta_w = 0.0
		self.theta_w_1 = 0.0

		self.enc_yaw = 0.0

		self.Dx = 0.0
		self.Dy = 0.0
		self.X = 0.0
		self.Y = 0.0

		#KALMAN VARIABLES
		self.kf_x = 0.0
		self.kf_y = 0.0
		self.kf_theta = 0.0

		self.rate = rospy.Rate(20)

		self.tiempo = 0.0

		self.yaw = 0.0
		self.yaw_1 = 0.0
		self.samples = 0
		#rospy.spin()

	def imu(self,data):
		self.yaw = data.orientation.x

	def KF_correct(self,data):
		self.kf_x = data.pose.pose.position.x
		self.kf_y = data.pose.pose.position.y
		self.kf_theta = data.pose.pose.position.z

        def read_enc(self,data):
		self.counter = data.velocity[5]
		self.deltatiempo = self.counter - self.counter_1

		self.PL = data.velocity[4]
		self.PR = data.velocity[3]

                if self.deltatiempo == 0:
                        self.deltatiempo = self.deltatiempo_1

		##================================================================================##
		#			POSICION A PARTIR DEL DESPLAZAMIENTO			   #
		##================================================================================##
		self.samples += 1

		if self.samples == 1:
			self.PL_1 = self.PL
			self.PR_1 = self.PR

		self.delta_SL = (self.PL - self.PL_1)*(2*np.pi*0.2)/1500.0
		self.delta_SR = (self.PR - self.PR_1)*(2*np.pi*0.2)/1500.0

                #POSICION
		self.delta_s = (self.delta_SR + self.delta_SL)/2.0;
		self.delta_theta = (self.delta_SR - self.delta_SL)/(2*self.yg)
		#self.theta = self.theta + self.delta_theta
		self.theta = self.kf_theta + self.delta_theta

		while self.theta > np.pi:
			self.theta -= 2*np.pi
		while self.theta < -np.pi:
			self.theta += 2*np.pi

		#self.delta_PX = self.delta_s * np.cos(self.theta + self.delta_theta/2.0)
		#self.delta_PY = self.delta_s * np.sin(self.theta + self.delta_theta/2.0)
		if self.samples <= 6:
			self.yaw_1 = self.yaw
		self.samples += 1

		self.delta_PX = self.delta_s * np.cos(self.yaw - self.yaw_1)
		self.delta_PY = self.delta_s * np.sin(self.yaw - self.yaw_1)

		self.PX = self.PX + self.delta_PX
		self.PY = self.PY + self.delta_PY

		#self.PX = self.kf_x + self.delta_PX
		#self.PY = self.kf_y + self.delta_PY

		##=================================================================================##
		#			FIN DE LA POSICION A PARTIR DEL DESPLAZAMIENTO		    #
		##=================================================================================##

		##=================================================================================##
                #			ACTUALIZACION DE VARIABLES				    #
		##=================================================================================##
		self.PL_1 = self.PL # recordar que aca iban las data.velocity[i]
		self.PR_1 = self.PR

                self.counter_1 = self.counter
                self.deltatiempo_1 = self.deltatiempo

                self.Left_1 = self.Left
                self.Right_1 = self.Right

                self.W_Left_1 = self.W_Left
                self.W_Right_1 = self.W_Right
		## ================================================================================##
		##			FIN DE LA ACTUALIZACION					    #
		##=================================================================================##


	def main(self):

		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			#print("Delta LEFT",self.delta_SL)
			#print("Delta RIGHT", self.delta_SR)
			#print("sample",self.samples)
			#print("Delta S", self.delta_s)
			#print self.samples
			#print("POS X", self.PX)
			#print("POS Y", self.PY)
			#print("theta_w", self.theta)
			#print("Dtheta", self.Dtheta)
			#print("DX", self.Dx)
			#print("DY", self.Dy)
			#print("WL",self.W_Left)
			#print("WR",self.W_Right)
			#print(" ")

			odom.header.stamp = rospy.get_rostime()
			odom.header.frame_id = "ODOMETRIA ENCODERS"

			odom.twist.twist.angular.x = self.WL # Velocidad angular motor izquierdo
			odom.twist.twist.angular.y = self.WR # Velocidad angular motor derecho

			odom.twist.twist.linear.x = self.WL * 0.2 #VELOCIDAD LINEAL RUEDAS IZQUIERDA
			odom.twist.twist.linear.y = self.WR * 0.2 #VELOCIDAD LINEAL RUEDAS DERECHA

			odom.pose.pose.position.x = self.PX
			odom.pose.pose.position.y = self.PY
			odom.pose.pose.position.z = self.theta
			self.pub.publish(odom)

			rate.sleep()

if __name__=='__main__':
        try:
                rospy.init_node("odom_node")
                print "Nodo POS Creado"
                cv = ubication()
                cv.main()
        except rospy.ROSInterruptException:
                pass
