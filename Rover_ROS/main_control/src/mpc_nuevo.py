#! /usr/bin/env python
import serial
import rospy
import numpy as np
import pigpio
import sys

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

pi = pigpio.pi()
msg = JointState()

mpc = JointTrajectoryControllerState()

class adq_datos(object):

        def __init__(self):
		self.pub = rospy.Publisher('/main_node', JointState, queue_size=10)
		self.pub1 = rospy.Publisher('/MPC', JointTrajectoryControllerState, queue_size=10)

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

		self.count = 0

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

		#self.goalx = [0.0, 5.0, 5.0, 1.0, 1.0, 5.0, 0.0]
		#self.goaly = [0.0, 0.0, -3.0, -3.0, -6.0, -6.0, 0.0]
		self.goalx = [0.0, 0.0, 5.0, 5.0, 0.0] #, 1.0, 0.0]
		self.goaly = [0.0, -10.0, -10.0, 0.0, 0.0] #, -5.0, 0.0]
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

		#self.A = np.array([[2.292, -1.61, 0.3177],
                #		[1, 0, 0],
                #		[0, 1, 0]])

		#self.B = np.array([ [1],[0],[0] ])
		#self.C = np.array([ [5.937e-05, 0.0001825, 3.355e-05] ])

		self.A = np.array([[0.9494, -0.07311, 0],
				[0.04874, 0.9982, 0],
				[0.001229, 0.04997, 0]])

		self.B = np.array([ [0.04874],[0.001229],[2.057e-05] ])
		self.C = np.array([ [0, 0, 1] ])

		self.Np = 3
		self.Nc = 2

		##======================= MODELO EXTENDIDO =================================##
		[self.m,self.n] = self.C.shape
		[self.n,self.p] = self.B.shape

		self.M_e = np.zeros([self.n+self.p,self.n+self.p])
		self.M_e[0:self.n,0:self.n] = self.A
		self.M_e[0:self.n,[self.n]] = self.B
		self.M_e[self.n,0:self.n] = np.zeros([self.p,self.n])
		self.M_e[self.n,self.n] = np.identity(self.p)

		self.N_e = np.zeros([self.n+self.p, self.p])
		self.N_e[0:self.n,0:self.p] = self.B
		self.N_e[self.n:self.n+self.p,0:self.p] = np.identity(self.p)

		self.Q_e = np.zeros([self.m,self.n+self.p])
		self.Q_e[0:self.m,0:self.n] = self.C
		self.Q_e[0:self.m,self.n:self.n+self.p-1] = np.zeros([self.m,self.p])

		#print ("M",self.M_e)
		#print ("N",self.N_e)
		#print ("Q",self.Q_e)

		#[self.n,self.n_in] = self.B_e.shape
		#self.xm = np.zeros(self.B.shape)
		#self.Xf = np.zeros([self.n,1])
		self.r = 0		#SET POINT

		self.Rk = 1 	# PESOS DE LA SALIDA
		self.Qk = 0.1	# PESO DE LA ACCION DE CONTROL

		self.Rw = np.dot(self.Rk,np.identity(self.Np))
		self.Qw = np.dot(self.Qk,np.identity(self.Nc))


		## =============== VARIABLES DE LA ACCION DE CONTROL ========================##

		self.xk = np.zeros([self.n+1,1])
		self.Zk = np.zeros([self.n+1,1])

		self.uk = 0.0
		self.uk_1 = 0.0

		[self.m1, self.n1] = self.Q_e.shape
		self.p1 = 1
		self.MN2 = np.identity(self.n1)
		self.F = np.zeros([self.Np,self.n1])

		for i in range(0, self.Np):
			self.MN2 = np.dot(self.MN2,self.M_e)
			self.F[i,:] = np.dot(self.Q_e,self.MN2)

		self.H = np.zeros([self.Np,self.Nc])
		self.H1 = np.zeros([self.Np,1])

		for i in range(1, self.Nc+1):
			if i == 1:
				self.H1 = np.vstack((self.Q_e,self.F[0:self.Np-i]))
			else:
				self.H1 = np.vstack((np.zeros([i-1,self.n1]),self.Q_e,self.F[0:self.Np-i]))

			self.H[:,[i-1]] = np.dot(self.H1,self.N_e)

		K1 = np.dot(self.H.T,self.Rw)
		K2 = np.dot(K1,self.H) + self.Qw

		self.K = np.dot(np.linalg.inv(K2),K1)
		#print self.K

		#self.Ky = np.dot(np.linalg.inv(self.Phi_Phi+2.5*np.identity(self.Nc)), self.Phi_R)
		#self.Ky = self.Ky = self.Ky[0,0]

		#self.K_mpc = np.dot(np.linalg.inv(self.Phi_Phi+2.5*np.identity(self.Nc)), self.Phi_F)
		#self.K_mpc = np.array(self.K_mpc[0,:], ndmin=2)

		#self.u = 0.0
		#self.deltaU = 0.0
		#self.deltau = 0.0
		#self.y = 0.0
		#self.yk = 0.0
		#self.yk_1 = 0.0
		##===========================================================================##
		# 			FIN DE LAS VARIABLES DEL CONTROLADOR		      #
		##===========================================================================##

		## VARIABLES DEL CONTROL DE VELOCIDAD
		self.Vd = 0.0


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

		# DIRECCION VECTOR TRAYECTORIA
		delta_wpx = self.wpx_2 - self.wpx_1
		delta_wpy = self.wpy_2 - self.wpy_1

		self.alpha = np.arctan2(delta_wpy, delta_wpx)

		# ============================== INTERPUNTO ===================================##
		self.PT_x = self.Ts*1.0*self.contador*np.cos(self.alpha) + self.wpx_1
		self.PT_y = self.Ts*1.0*self.contador*np.sin(self.alpha) + self.wpy_1
		#self.PT_x = self.Ts*1.5*self.contador*np.cos(self.alpha) + self.wpx_1
		#self.PT_y = self.Ts*1.5*self.contador*np.sin(self.alpha) + self.wpy_1
		##=============================================================================##

		#self.contador = self.contador + 1

		# ANGULO DE CONTROL
		self.angle_to_goal = np.arctan2(self.PT_y - self.pos_y, self.PT_x - self.pos_x)

		## ====================== DISTANCIA AL PUNTO ==================================##
                delta_x1 = self.PT_x - self.pos_x
                delta_y1 = self.PT_y - self.pos_y
                #delta_x2 = self.wpx_2 - self.pos_x
                #delta_y2 = self.wpy_2 - self.pos_y
		delta_x2 = self.wpx_2 - self.PT_x
		delta_y2 = self.wpy_2 - self.PT_y

                absolute_x1 = np.power(abs(delta_x1),2)
                absolute_y1 = np.power(abs(delta_y1),2)
                absolute_x2 = np.power(abs(delta_x2),2)
                absolute_y2 = np.power(abs(delta_y2),2)

                self.distancia = np.sqrt(absolute_x2 + absolute_y2)

		# error control de velocidad
                self.distancia_PT = np.sqrt(absolute_x1 + absolute_y1)

		# CONTROL DE VELOCIDAD PROPORCIONAL
		k = 1.2
		self.Vd = k*self.distancia_PT
		#self.Vd = 0.5

		if self.Vd > 2.0:
			self.Vd = 2.0

		# ERROR GENERADOR DE INTERPUNTOS
                if self.distancia <= 0.5:
                        self.ini = self.ini + 1
                        #REINICIO DEL CONTADOR
                        #PARA NO SUMAR DOS VECES LA POS ANTERIOR
                        self.contador = 1

		#if self.ini >= 4:
		#	self.ini = 4

		# ERROR DEL WAYPOINT
                #if self.distancia_PT <= 1.5:
                #        self.contador = self.contador + 1

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
                if self.RC1 >= 254.0:
                        self.RC1 = 254.0

                if self.RC2 >= 254.0:
                        self.RC2 = 254.0

                if self.RC1 <= 1.0:
                        self.RC1 = 1.0

                if self.RC2 <= 1.0:
                        self.RC2 = 1.0

                # SENAL DE PWM A LOS CANALES DEL ROBOTEQ
                pi.set_PWM_dutycycle(13, self.RC1) # AVANZAR
                pi.set_PWM_dutycycle(12, self.RC2) # GIRAR

                # PUBLICAR MENSAJE
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
		if 1000 < self.calibration < 1200:
			pi.set_PWM_dutycycle(13, 127.5)
			pi.set_PWM_dutycycle(12, 127.5)

		#	if self.calibra < 54:
		#		pi.set_PWM_dutycycle(12,180)
		#		print "calibrando"

		#	self.calibra = self.calibra + 1

		#	if self.calibra == 54:
		#		pi.set_PWM_dutycycle(12, 125.5)
		#		pi.set_PWM_dutycycle(13, 125.5)
		#		msg.name = ['Termino']
		#		self.pub.publish(msg)
		#		msg.name = []

		##================================================================================##
		# 			MODO CONTROL PREDICTIVO					   #
		##================================================================================##
		if 1900 < self.calibration < 2000:
			#pi.set_PWM_dutycycle(13, 125.5)
			#pi.set_PWM_dutycycle(12, 125.5)

			# VARIABLES DEL VECTOR DE ESTADO THETA,THETA',THETA''
			self.delta_pos = self.yaw - self.yaw_1
			self.delta_w = self.w - self.w_1

			self.acc = self.delta_w/self.Ts
			self.delta_acc = self.acc - self.acc_1

			##=====================================================================##
			#			CONTROL PREDICTIVO				#
			##=====================================================================##
			#self.count += 1
			#if self.count >= 1:
			#	self.r = -1.5 #self.angle_to_goal
			#if self.count >= 100:
			#	self.r = 3.0
			#if self.count >= 200:
			#	self.r = 1.5
			#if self.count >= 300:
			#	self.r = -0.1

			self.Zk[0,0] = self.xk[0,0]
			self.Zk[1,0] = self.xk[1,0]
			self.Zk[2,0] = self.xk[2,0]
			self.Zk[3,0] = self.uk_1

			#self.Zk[0:self.n,0] = self.xk
			#self.Zk[self.n,0] = self.uk_1

			#print self.Zk.shape
			#print self.K.shape
			#print self.F.shape

			self.r = -1.50 #self.angle_to_goal

			error1 = self.r - self.yaw
                        if error1 > np.pi:
                                self.r = self.r - 2*np.pi
                        if error1 < -np.pi:
                                self.r = self.r + 2*np.pi

			# ACCION DE CONTROL
			du1 = self.r - np.dot(self.F,self.Zk)
			self.deltaU =  np.dot(self.K, du1)
			self.deltau = self.deltaU[0,0]
			self.uk = self.deltau + self.uk_1

			#print self.uk

			print self.xk.shape

			self.yk = self.yaw

			self.xk[0,0] = self.delta_acc
			self.xk[1,0] = self.delta_w
			self.xk[2,0] = self.delta_pos
			self.xk[3,0] = self.yk


			# SELECCION DEL ANGULO ADECUADO PARA UN ERROR MAS PEQUENO
			error1 = self.r - self.yk
			#if error1 > np.pi:
			#	self.xk[3,0] = self.yk - 2*np.pi
			#if error1 < -np.pi:
			#	self.xk[3,0] = self.yk + 2*np.pi

			#if error1 < -np.pi and self.yk > np.pi/2.0:
			#	self.xk[3,0] = error1
			#if error1 > np.pi and self.yk < -np.pi/2.0:
			#	self.xk[3,0] = error1

			#if error1 > np.pi:
			#	self.r = self.r - 2*np.pi
			#if error1 < -np.pi:
			#	self.r = self.r + 2*np.pi

			# LINEALIZAR LA ACCION DE CONTROL A PWM
			uk = 31.75*self.uk + 127
			if uk >= 254.0:
				uk = 254.0
			if uk <= 5.0:
				uk = 5.0

			#if self.deltaU >= 254.0:
			#	self.deltaU = 254.0
			#if self.deltaU <= 0.1:
			#	self.deltaU = 0.1

                        #if self.u >= 254.0:
                        #        self.u = 254.0
                        #if self.u <= 0.1:
                        #        self.u = 0.1

			self.manual()

			# LINEALIZAR LA ACCION DE CONTROL DE VELOCIDAD A PWM
			Vd = -36.666*self.Vd + 127
			if Vd >= 127.0:
				vd = 127.0
			if Vd <= 72.0:
				Vd = 72.0

                        if self.ini + 2 > len(self.goalx):
                                pi.set_PWM_dutycycle(12, 127.5)
                                pi.set_PWM_dutycycle(13, 127.5)
                                #sys.exit(0)
                        else:
                                pi.set_PWM_dutycycle(12, uk)
                                pi.set_PWM_dutycycle(13, 127.5)

			#pi.set_PWM_dutycycle(12, du)
			#pi.set_PWM_dutycycle(13, Vd)
			#pi.set_PWM_dutycycle(12, self.RC2)
			#pi.set_PWM_dutycycle(13, self.RC1)

			self.contador = self.contador + 1

			## ====================================================================##

			# ACTUALIZACION DE VARIABLES
			self.yaw_1 = self.yaw
			self.w_1 = self.w
			self.yk_1 = self.yk
			self.acc_1 = self.acc
			self.uk_1 = self.uk

			#print("control u", self.u)
			#print("Control_delta", self.deltaU)
			#print("du", du)
			#print("Angle_goal", self.r)
			#print("Angle", self.yk)
			#print("xk_theta", self.xk[3,0])
			#print " "

			mpc.header.stamp = rospy.get_rostime()
			mpc.desired.positions = [self.PT_x, self.PT_y, self.wpx_1, self.wpy_1, self.wpx_2, self.wpy_2]
			mpc.desired.effort = [self.r]

			mpc.actual.positions = [self.pos_x, self.pos_y, self.yaw]
			mpc.actual.velocities = [Vd, self.Vd]
			mpc.actual.effort = [uk, self.uk]

			mpc.error.positions = [self.distancia_PT, self.distancia]
			mpc.error.effort = [error1]

			self.pub1.publish(mpc)

                	print("angle_goal",self.r)
                	print("yaw",self.yaw)
                	#print("angulo_coordenada", self.alpha)
			print("speed", Vd)
                	print("control_MPC_U", uk)
                	#print("control_MPC_U", self.u)
                	#print("error1:",error1)
                	#print("error2:",error2)
                	print("error1:",error1)
			print("xk", self.xk[3,0])
                	print("self_yk", self.yk)
                	#print("error_1", self.error_1)
                	#print("comple:", self.comp)
                	print("wp1", self.wpx_1,self.wpy_1)
                	print("wp2", self.wpx_2,self.wpy_2)
                	print("rover", self.pos_x,self.pos_y)
                	print("PT_X PT_Y", self.PT_x, self.PT_y)
                	print("distancia_PT0", self.distancia_PT)
                	print("distancia", self.distancia)
                	print("ini", self.ini)
                	print("contador", self.contador)
                	print " "


	def main(self):
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			if 1900 < self.modo < 2200:
				self.stop()

			if 1400 < self.modo < 1600:
				self.manual()

			if 900 < self.modo < 1000:
				self.automatico()

			rate.sleep()

if __name__=='__main__':
       	try:
		rospy.init_node("main_control")
		cv = adq_datos()
		print "Nodo PWM creado"
		cv.main()

	except AttributeError, a:
		print "error_de_atributo: " + str(a)
		pi.set_PWM_dutycycle(13, 125.5)
		pi.set_PWM_dutycycle(12, 125.5)

	except pigpio.error:
                print "error_pigpio"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except IndexError, c:
                print "error_IndexdeLista: " + str(c)
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

	except NameError, e:
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)
		print "NameError: " + str(e)


