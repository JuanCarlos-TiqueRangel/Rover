#! /usr/bin/env python
import serial
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

odom = Odometry()

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/position', Odometry, queue_size=10)

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

		self.SL = 0.0
		self.SR = 0.0

		self.PX = 0.0
		self.PY = 0.0

		self.theta = 0.0
		self.yg = 0.38
		self.delta_s = 0.0
		self.base_width = 0.6

		rospy.Subscriber('/enc_data', JointState, self.read_enc)

        def read_enc(self,data):
		self.counter = data.velocity[5]
		self.deltatiempo = self.counter - self.counter_1

		if self.deltatiempo == 0:
			self.deltatiempo = self.deltatiempo_1

		# Angular Velocity rad/s and Lineal Velocity m/s
		# 1500 son pulsos por RPM que tiene el encoder

		self.W_Left = (data.velocity[3]-self.PL_1)*(2*np.pi)/(1500*self.enc_ts*self.deltatiempo)
		self.W_Right = (data.velocity[4]-self.PR_1)*(2*np.pi)/(1500*self.enc_ts*self.deltatiempo)

		#saturar la aceleracion
		if abs(self.W_Left - self.W_Left_1) > 25.0 or abs(self.W_Right - self.W_Right_1) > 25.0:
			self.W_Left = self.W_Left_1
			self.W_Right = self.W_Right_1

		#filtro buttter
		self.Left = self.W_Left
		self.Right = self.W_Right
		self.WL = 0.2452*self.Left + 0.2452*self.Left_1 + 0.5095*self.WL
		self.WR = 0.2452*self.Right + 0.2452*self.Right_1 + 0.5095*self.WR

		#DESPLAZAMIENTO
		self.SL += (data.velocity[3]-self.PL_1)*(2*np.pi*0.2)/1500.0 #Desplazamiento ruedas izquierdas
		self.SR += (data.velocity[4]-self.PR_1)*(2*np.pi*0.2)/1500.0 #Desplazamiento ruedas derechas

		#POSICION
		self.delta_s = (self.SR + self.SL)/2
		#self.theta = (self.SR - self.SL)/self.yg
		self.theta = (self.SR - self.SL)/self.base_width

		while self.theta > np.pi:
			self.theta -= np.pi
		while self.theta < -np.pi:
			self.theta += np.pi

		self.PX = self.delta_s * np.cos(self.theta + (self.SR - self.SL)/(2*self.yg))
		self.PY = self.delta_s * np.sin(self.theta + (self.SR - self.SL)/(2*self.yg))

		#update variables
		self.counter_1 = self.counter
		self.deltatiempo_1 = self.deltatiempo

		self.PL_1 =  data.velocity[3]
		self.PR_1 = data.velocity[4]

		self.Left_1 = self.Left
		self.Right_1 = self.Right

		self.W_Left_1 = self.W_Left
		self.W_Right_1 = self.W_Right

		#print self.WL*9.5493
		#print self.WR*9.5493

		#print ("POS_X",self.PX)
		#print ("pos_Y",self.PY)
		#print ("yaw",self.theta)
		#print " "

	def main(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			odom.header.stamp = rospy.get_rostime()
               		odom.twist.twist.angular.x = self.WL # Velocidad angular motor izquierdo
               		odom.twist.twist.angular.y = self.WR # Velocidad angular motor derecho

               		odom.pose.pose.position.x = self.PX #Posicion en x generada por los encoders
                	odom.pose.pose.position.y = self.PY #Posicion en y generada por los encoders
	               	odom.pose.pose.position.z = self.theta #Angulos generado por los encoders

			self.pub.publish(odom)
			rate.sleep()


if __name__=='__main__':
        try:
                rospy.init_node('Position',anonymous=True, disable_signals=True)
		print "Nodo POS Creado"
                cv = ubication()
		cv.main()
        except rospy.ROSInterruptException:
                pass

