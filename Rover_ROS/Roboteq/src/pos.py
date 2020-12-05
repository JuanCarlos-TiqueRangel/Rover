#! /usr/bin/env python
import serial
import rospy
import numpy as np

from math import pi, cos, sin
from sensor_msgs.msg import JointState
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

		#rospy.spin()

	def KF_correct(self,data):
		self.kf_x = data.pose.pose.position.x
		self.kf_y = data.pose.pose.position.y
		self.kf_theta = data.pose.pose.position.z

        def read_enc(self,data):
		self.counter = data.velocity[5]
		self.deltatiempo = self.counter - self.counter_1

		self.PL = data.velocity[3]
		self.PR = data.velocity[4]

                if self.deltatiempo == 0:
                        self.deltatiempo = self.deltatiempo_1

                ##===========================================================================================##
                #       		POSICION X,Y A PARTIR DE LA VELOCIDAD ANGULAR			      #
                ## ==========================================================================================##

                self.W_Left = (self.PL - self.PL_1)*(2*np.pi)/(1500*self.enc_ts)
                self.W_Right = (self.PR - self.PR_1)*(2*np.pi)/(1500*self.enc_ts)

                #saturar la aceleracion
                if abs(self.W_Left - self.W_Left_1) > 25.0 or abs(self.W_Right - self.W_Right_1) > 25.0:
                        self.W_Left = self.W_Left_1
                        self.W_Right = self.W_Right_1

                #filtro buttter velocidad
                self.Left = self.W_Left
                self.Right = self.W_Right
                self.WL = 0.2452*self.Left + 0.2452*self.Left_1 + 0.5095*self.WL
                self.WR = 0.2452*self.Right + 0.2452*self.Right_1 + 0.5095*self.WR

		self.Ds = (self.enc_ts*0.2/2)*(self.WR + self.WL)
		self.Dtheta = (self.enc_ts*0.2/(2*0.38))*(self.WR - self.WL)
		self.theta_w = self.kf_theta + self.Dtheta

		#self.theta_w = self.theta_w + self.Dtheta

		while self.theta_w > np.pi:
			self.theta_w -= 2*np.pi

		while self.theta_w < -np.pi:
			self.theta_w += 2*np.pi

		self.Dx = self.Ds*np.cos(self.theta_w + self.Dtheta/2)
		self.Dy = self.Ds*np.sin(self.theta_w + self.Dtheta/2)

		self.X = self.kf_x + self.Dx
		self.Y = self.kf_y + self.Dy

		#self.X = self.X + self.Dx
		#self.Y = self.Y + self.Dy

		##================================================================================##
		#			FIN DE LA POSICION CON VELOIDAD ANGULAR			   #
		##================================================================================##


		##================================================================================##
		#			POSICION A PARTIR DEL DESPLAZAMIENTO			   #
		##================================================================================##
		self.samples += 1

		if self.samples == 1:
			self.PL_1 = self.PL
			self.PR_1 = self.PR

		DPL = self.PL - self.PL_1
		DPR = self.PR - self.PR_1

		if DPL >= 18:
			DPL = DPL - 6

		self.delta_SL = DPL*(2*np.pi*0.2)/1500.0
		self.delta_SR = DPR*(2*np.pi*0.2)/1500.0

                #POSICION
		self.delta_s = (self.delta_SR + self.delta_SL)/2.0;
		self.delta_theta = (self.delta_SR - self.delta_SL)/(2*self.yg)
		#self.theta = self.theta + self.delta_theta
		self.theta = self.kf_theta + self.delta_theta

		if self.theta > np.pi:
			self.theta -= 2*np.pi
		if self.theta < -np.pi:
			self.theta += 2*np.pi

		self.delta_PX = self.delta_s * np.cos(self.theta + self.delta_theta/2.0)
		self.delta_PY = self.delta_s * np.sin(self.theta + self.delta_theta/2.0)

		#self.PX = self.PX + self.delta_PX
		#self.PY = self.PY + self.delta_PY

		self.PX = self.kf_x + self.delta_PX
		self.PY = self.kf_y + self.delta_PY

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
			#print ("delta_X", self.delta_PX)
			#print ("delta_Y", self.delta_PY)
			#print ("Dx_vel", self.Dx)
			#print ("Dy_vel", self.Dy)
			#print " "

			odom.header.stamp = rospy.get_rostime()
			odom.header.frame_id = "ODOMETRIA ENCODERS"

			odom.twist.twist.angular.x = self.WL # Velocidad angular motor izquierdo
			odom.twist.twist.angular.y = self.WR # Velocidad angular motor derecho

			odom.twist.twist.linear.x = self.delta_SL #self.WL * 0.2 #VELOCIDAD LINEAL RUEDAS IZQUIERDA
			odom.twist.twist.linear.y = self.delta_SR #self.WR * 0.2 #VELOCIDAD LINEAL RUEDAS DERECHA

			odom.pose.pose.position.x = self.PX
			odom.pose.pose.position.y = self.PY
			odom.pose.pose.position.z = self.theta
			self.pub.publish(odom)

			#print("LEFT", self.delta_SL)
			#print("RIGHT", self.delta_SR)
			#print " "

			rate.sleep()

if __name__=='__main__':
        try:
                rospy.init_node("odom_node")
                print "Nodo POS Creado"
                cv = ubication()
                cv.main()
        except rospy.ROSInterruptException:
                pass
