#! /usr/bin/env python
import serial
import rospy
import numpy as np
import pigpio

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

pi = pigpio.pi()
msg = JointState()


class adq_datos(object):

        def __init__(self):
		self.pub = rospy.Publisher('/main_node', JointState, queue_size=10)

		self.Ts = 0.05

		self.modo = 0
		self.calibration = 0
		self.th = 0
		self.st = 0
		self.refT = 1535
		self.refS = 1535

		self.RC1 = 0
		self.RC2 = 0
		self.cuenta = 0
		self.calibra = 0

		self.avanzar = 72.0

                # pins mode
                pi.set_mode(13, pigpio.OUTPUT)
                pi.set_mode(12, pigpio.OUTPUT)

                # hardware_PWM(gpio, PWMfreq, PWMduty)
                self.M1 = pi.hardware_PWM(12, 1000, 0)
                self.M2 = pi.hardware_PWM(13, 1000, 0)

		##=============================================================================##
		#			VARIABLES DE LA TRAYECTORIA 				#
		##==============================================================================#
		self.pos_x = 0.0
		self.pos_y = 0.0
		self.yaw = 0.0
		self.yaw_1 = 0.0

		self.wpx_1 = 0.0
		self.wpy_1 = 0.0
		self.wpx_2 = 0.0
		self.wpy_2 = 0.0

		self.alpha = 0.0

		self.PT_x = 0.0
		self.PT_y = 0.0

		self.contador = 0
		self.angle_to_goal = 0.0
		self.distancia = 0.0
		self.distancia_PT = 0.0
		self.goalx = [0,10,10,0,0,10,0]
		self.goaly = [0,0,5,5,10,10,0]
		self.ini = 0

		##===========================================================================##
		#			VARIABLES MODELO CONTROLADOR			      #
		##===========================================================================##
		self.delta_pos = 0.0
		self.w = 0.0
		self.w_1 = 0.0
		self.delta_w = 0.0
		self.acc = 0.0
		self.acc_1 = 0.0
		self.delta_acc = 0.0

		self.A = np.array([[0.4534, -2.705, 0],
                		[0.07243, 0.8477, 0],
                		[0.00408, 0.09464, 1]])

		self.B = np.array([ [0.07243],[0.00408],[0.0001437] ])
		self.C = np.array([ [0,0,65.7] ])

		self.Np = 24
		self.Nc = 25

		##======================= MODELO EXTENDIDO =================================##
		[self.m1,self.n1] = self.C.shape
		[self.n1,self.n_in] = self.B.shape
		self.A_e = np.identity(self.n1+self.m1)
		self.A_e[0:self.n1,0:self.n1] = self.A
		self.A_e[self.n1:self.n1+self.m1,0:self.n1] = np.dot(self.C,self.A)
		self.B_e = np.zeros([self.n1+self.m1,self.n_in])
		self.B_e[0:self.n1,:] = self.B
		self.B_e[self.n1:self.n1+self.m1,:] = np.dot(self.C,self.B)
		self.C_e = np.zeros([self.m1,self.n1+self.m1])
		self.C_e[:,self.n1:self.n1+self.m1] = np.ones([self.m1,self.m1])

		[self.n,self.n_in] = self.B_e.shape
		self.xm = np.zeros(self.B.shape)
		self.Xf = np.zeros([self.n,1])
		self.r = 0		#SET POINT

		self.h = self.C_e
		self.F = np.dot(self.C_e,self.A_e)

		for kk in range(1, self.Np):
			self.h = np.vstack((self.h,np.dot(self.h[kk-1,:],self.A_e)))
			self.F = np.vstack((self.F,np.dot(self.F[kk-1,:],self.A_e)))

		self.v = np.dot(self.h,self.B_e)
		self.Phi = np.zeros([self.Np,self.Nc]); #declare the dimension of Phi
		self.Phi[:,0] = self.v[:,0]

		for i in range(1, self.Nc):
			self.Phi[i:,i] = self.v[0:self.Np-i:,0]

		self.BarRs = np.ones([self.Np,1])
		self.Phi_Phi = np.dot(self.Phi.T,self.Phi)
		self.Phi_F = np.dot(self.Phi.T,self.F)
		self.Phi_R = np.dot(self.Phi.T,self.BarRs)
		self.xk = np.zeros(self.B_e.shape)
		self.xk_1 = np.zeros(self.B_e.shape)
		self.u1 = 0

		## =============== VARIABLES DE LA ACCION DE CONTROL ========================##
		self.Ky = np.dot(np.linalg.inv(self.Phi_Phi+2.5*np.identity(self.Nc)), self.Phi_R)
		self.Ky = self.Ky = self.Ky[0,0]

		self.K_mpc = np.dot(np.linalg.inv(self.Phi_Phi+2.5*np.identity(self.Nc)), self.Phi_F)
		self.K_mpc = np.array(self.K_mpc[0,:], ndmin=2)

		self.u = 0.0
		self.deltaU = 0.0
		self.y = 0.0
		self.yk = 0.0
		self.yk_1 = 0.0
		##===========================================================================##
		# 			FIN DE LAS VARIABLES DEL CONTROLADOR		      #
		##===========================================================================##

                rospy.Subscriber('/channels', JointState, self.synchronize_pwm)
		rospy.Subscriber("/kalman_filter", Odometry, self.trajectory)



	def trajectory(self, data):

		## ================ UBICACION Y ORIENTACION ==================================##
		self.pos_x = data.pose.pose.position.x
		self.pos_y = data.pose.pose.position.y
		self.yaw = data.pose.pose.position.z

		self.w = data.twist.twist.angular.z
		## ===========================================================================##

		# WAYPOINTS
		self.wpx_1 = self.goalx[self.ini]
		self.wpy_1 = self.goaly[self.ini]
		self.wpx_2 = self.goalx[self.ini+1]
		self.wpy_2 = self.goaly[self.ini+1]

		# VECTOR DISTANCIA
		delta_wpx = self.wpx_2 - self.wpx_1
		delta_wpy = self.wpy_2 - self.wpy_1

		self.alpha = np.arctan2(delta_wpy, delta_wpx)

		# ============================== INTERPUNTO ===================================##
		self.PT_x = self.Ts*0.2*self.contador*np.cos(self.alpha) + self.wpx_1
		self.PT_y = self.Ts*0.2*self.contador*np.sin(self.alpha) + self.wpy_1
		##=============================================================================##

		# ANGULO DE CONTROL
		self.angle_to_goal = np.arctan2(self.PT_y - self.pos_y, self.PT_x - self.pos_x)

		## ====================== DISTANCIA AL PUNTO ==================================##
                delta_x1 = self.PT_x - self.pos_x
                delta_y1 = self.PT_y - self.pos_y
                delta_x2 = self.wpx_2 - self.pos_x
                delta_y2 = self.wpy_2 - self.pos_y

                absolute_x1 = np.power(abs(delta_x1),2)
                absolute_y1 = np.power(abs(delta_y1),2)
                absolute_x2 = np.power(abs(delta_x2),2)
                absolute_y2 = np.power(abs(delta_y2),2)

                self.distancia = np.sqrt(absolute_x2 + absolute_y2)
                self.distancia_PT = np.sqrt(absolute_x1 + absolute_y1)

                if self.distancia <= 0.5:
                        self.ini = self.ini + 1
                        #REINICIO DEL CONTADOR
                        #PARA NO SUMAR DOS VECES LA POS ANTERIOR
                        self.contador = 1

                if self.distancia_PT <= 0.5:
                        self.contador = self.contador + 1


        def synchronize_pwm(self, pwm):
		self.modo = pwm.velocity[0]		# AX1
		self.calibration = pwm.velocity[1]	# AX2
               	self.st = pwm.velocity[2]		# Stering
               	self.th = pwm.velocity[3]		# Throttle

	def stop(self):
		x = 1
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

		print "STOP"

	def manual(self):

                delta = self.th - self.refT
                if abs(delta) > 600:
                        self.th = self.refT

                delta = self.st - self.refS
                if abs(delta) > 600:
                        self.st = self.refS

                #Ecuaciones para linealizar y aplicar un PWM de 0 a 255
                self.RC1 = (2095-self.th)/1130.0 * 255
                self.RC2 = (1920-self.st)/795.0 * 255

                #Saturacion de a los motores
                if self.RC1 >= 254.9:
                        self.RC1 = 254.9

                if self.RC2 >= 254.9:
                        self.RC2 = 254.9

                if self.RC1 <= 0.1:
                        self.RC1 = 0.1

                if self.RC2 <= 0.1:
                        self.RC2 = 0.1

                #Aplicar PWM a los motores
                pi.set_PWM_dutycycle(13, self.RC1)
                pi.set_PWM_dutycycle(12, self.RC2)

                #Publicar mensaje
                msg.velocity = [self.RC1,self.RC2]
		#msg.name = ['Hola']
                msg.header.stamp = rospy.get_rostime()
                self.pub.publish(msg)

                # update variables
                self.refT = self.th
                self.refS = self.st

                #print str(self.RC1) + "\t" + str(self.RC2)

	def automatico(self):
		#CALIBRAR MAGNETOMETRO
		if 1900 < self.calibration < 2200:
			if self.calibra < 54:
				pi.set_PWM_dutycycle(12,180)
				print "calibrando"

			self.calibra = self.calibra + 1

			if self.calibra == 54:
				pi.set_PWM_dutycycle(12, 125.5)
				pi.set_PWM_dutycycle(13, 125.5)
				msg.name = ['Termino']
				self.pub.publish(msg)
				msg.name = []

		##================================================================================##
		# 			MODO CONTROL PREDICTIVO					   #
		##================================================================================##
		if 900 < self.calibration < 1000:
			pi.set_PWM_dutycycle(13, 125.5)
			pi.set_PWM_dutycycle(12, 125.5)


			# VARIABLES DEL VECTOR DE ESTADO THETA,THETA',THETA''
			self.delta_pos = self.yaw - self.yaw_1
			self.delta_w = self.w - self.w_1

			self.acc = self.delta_w/self.Ts
			self.delta_acc = self.acc - self.acc_1

			##=====================================================================##
			#			CONTROL PREDICTIVO				#
			##=====================================================================##
			self.r = 2.15
			self.deltaU = self.Ky*self.r - np.dot(self.K_mpc, self.xk)
			self.u = self.deltaU + self.u

			self.yk = self.yaw

			self.xk[0,0] = self.delta_pos
			self.xk[1,0] = self.delta_w
			self.xk[2,0] = self.delta_acc
			self.xk[3,0] = self.yk
			## ====================================================================##

			# ACTUALIZACION DE VARIABLES
			self.yaw_1 = self.yaw
			self.w_1 = self.w
			self.yk_1 = self.yk
			self.acc_1 = self.acc

			print("Control", self.deltaU[0,0])
			print("Angle", self.yk)
			print " "

	def main(self):
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			if 1900 < self.modo < 2100:
				self.stop()

			elif 1400 < self.modo < 1600:
				self.manual()

			elif 900 < self.modo < 1000:
				self.automatico()

			rate.sleep()

if __name__=='__main__':
       	try:
		rospy.init_node("main_control")
		cv = adq_datos()
		print "Nodo PWM creado"
		cv.main()

	except AttributeError:
		print "error_de_atributo"
		pi.set_PWM_dutycycle(13, 125.5)
		pi.set_PWM_dutycycle(12, 125.5)

	except pigpio.error:
                print "error_pigpio"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except IndexError:
                print "error_IndexdeLista"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except rospy.ROSInterruptException:
		print "ROSInterrupt"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except KeyboardInterrupt:
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)
		print "keyboardInterrupt"

	except SyntaxError:
		pi.set_PWM_dutycycle(13, 125.5)
		pi.set_PWM_dutycycle(12, 125.5)
		print "SyntaxError"

