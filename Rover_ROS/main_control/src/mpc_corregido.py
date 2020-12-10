#! /usr/bin/env python
import traceback
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

		self.Ts = 0.2

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
		#self.angle_to_goal = 0.0
		self.distancia = 0.0
		self.distancia_PT = 0.0

		#self.goalx = [0.0, -10.0, -1.0, 0.0] #1.0, 1.0, 10.0, 0.0]
		#self.goaly = [0.0, -10.0, -15.0, 0.0] #-5.0, -10.0, -10.0, 0.0]

                #self.goalx = [0.0, -10.0, -10.0, 0.0]
                #self.goaly = [0.0, 0.0, -10.0, 0.0]

		self.goalx = [0.0, 10.0, 10.0, 1.0, 1.0, 10.0, 0.0] #, 1.0, 0.0]
		self.goaly = [0.0, 0.0, -5.0, -5.0, -10.0, -10.0, 0.0] #, -5.0, 0.0]

                #self.goalx = [0.0, -5.0, -1.0, 6.0, 10.0, -3.0, 1.0, 10.0]
                #self.goaly = [0.0, -5.0, -9.0, -2.0, -6.0, -19.0, -23.0, -14.0]

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

		self.Ad = np.array([[0.0234, -1.8745, 0],
				[0.0799, 0.7381, 0],
				[0.0112, 0.1797, 1]])

		self.Bd = np.array([ [0.0464],[0.0065],[0.0005] ])
		self.Cd = np.array([ [0, 0, 1] ])

		self.Np = 30
		self.Nc = 25

		##======================= MODELO EXTENDIDO =================================##
		[self.n,self.p] = self.Bd.shape
		[self.m,self.n_in] = self.Cd.shape

		self.M = np.identity(self.n+self.m)
		self.M[0:self.n,0:self.n] = self.Ad
		self.M[0:self.n,self.n] = self.Bd[:,0]

		self.N = np.zeros([self.n+self.p,self.p])
		self.N[0:self.n,0] = self.Bd[:,0]
		self.N[self.n,0] = 1

		self.Q = np.zeros([self.p,self.n+self.p])
		self.Q[0,0:self.n] = self.Cd

		self.Rk = 2.5
		self.Qk = 0.0006

		self.Rw = np.dot(self.Rk,np.identity(self.Np))
		self.Qw = np.dot(self.Qk,np.identity(self.Nc))

		[self.m1,self.n1] = self.Q.shape
		self.p1 = 1
		self.MN2 = np.identity(self.n1)
		self.F = np.zeros([self.Np,self.n1])

		for i in range(0, self.Np):
			self.MN2 = np.dot(self.MN2,self.M)
			self.F[i,:] = np.dot(self.Q,self.MN2)

		self.H = np.zeros([self.Np,self.Nc])

		for k in range(0, self.Nc):
			self.H[k:,k] = self.F[0:self.Np-k,3]

		self.ku1 = np.dot(self.H.T,self.Rw)
		self.ku2 = np.dot(self.ku1,self.H) + self.Qw
		self.ku = np.dot(np.linalg.inv(self.ku2),self.ku1)

		#print ("H",self.H)
		#print ("F",self.F)
		#print ("Q",self.Q)

		## =============== VARIABLES DE LA ACCION DE CONTROL ========================##

		self.xk = np.zeros([self.Ad.shape[0],1])
		self.xk_1 = np.zeros([self.Ad.shape[0],1])

		self.zk = np.zeros(self.N.shape)

		self.uk = 0.0
		self.uk_1 = 0.0

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

		if self.yaw > 3.0892:
			self.yaw = np.pi
		if self.yaw < -3.0892:
			self.yaw = -np.pi
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
		self.PT_x = self.Ts*1.3*self.contador*np.cos(self.alpha) + self.wpx_1
		self.PT_y = self.Ts*1.3*self.contador*np.sin(self.alpha) + self.wpy_1
		#self.PT_x = self.Ts*1.5*self.contador*np.cos(self.alpha) + self.wpx_1
		#self.PT_y = self.Ts*1.5*self.contador*np.sin(self.alpha) + self.wpy_1
		##=============================================================================##

		#self.contador = self.contador + 1

		# ANGULO DE CONTROL
		#self.angle_to_goal = np.arctan2(self.PT_y - self.pos_y, self.PT_x - self.pos_x)
		self.r = np.arctan2(self.PT_y - self.pos_y, self.PT_x - self.pos_x)
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
		k = 0.6
		self.Vd = k*self.distancia_PT
		#self.Vd = 0.5

		if self.Vd > 2.0:
			self.Vd = 2.0

		# ERROR GENERADOR DE INTERPUNTOS
                if self.distancia <= 0.2:
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

			if self.uk_1 > 127.5:
				self.uk_1 = 127.5
			if self.uk_1 < -127.5:
				self.uk_1 = -127.5

			self.zk[0,0] = self.xk[0,0]
			self.zk[1,0] = self.xk[1,0]
			self.zk[2,0] = self.xk[2,0]
			self.zk[3,0] = self.uk_1

			#self.Zk[0:self.n,0] = self.xk
			#self.Zk[self.n,0] = self.uk_1

			#print self.Zk.shape
			#print self.K.shape
			#print self.F.shape

			#self.r = self.angle_to_goal
			error1 = self.r - self.yaw
			print ("error 1: ",error1)
			print " "
			print ("r= ",self.r)
                        print ("yaw = ",self.yaw)
			if error1 > np.pi:
                                self.r = self.r - 2*np.pi
				print ">pi"
                        if error1 < -np.pi:
                                self.r = self.r + 2*np.pi
				print "<pi"
			print ("r corregido= ", self.r)
			print " "

			if (self.r > (178/np.pi)*180 and self.r < (182/np.pi)*180):
				self.r = np.pi
			elif (self.r < (-178/np.pi)*180 and self.r > (-182/np.pi)*180):
				self.r = -np.pi


			# ACCION DE CONTROL
			du1 = self.r - np.dot(self.F,self.zk)
			self.deltaU =  np.dot(self.ku[0,:], du1)
			self.deltau = self.deltaU[0]
			self.uk = self.uk_1 + self.deltau

			#print self.uk

			#print self.xk.shape

			self.yk = self.yaw

			self.xk[0,0] = self.acc
			self.xk[1,0] = self.w
			self.xk[2,0] = self.yaw

			self.manual()

			# LINEALIZAR LA ACCION DE CONTROL DE VELOCIDAD A PWM
			Vd = -36.666*self.Vd + 127
			if Vd >= 127.0:
				vd = 127.0
			if Vd <= 52.0:
				Vd = 52.0

			uk = self.uk + 127.5
			if uk >= 254.0:
				uk = 254.0
			if uk <= 0.0:
				uk = 1.0

                        if self.ini + 2 > len(self.goalx):
                                pi.set_PWM_dutycycle(12, 127.5)
                                pi.set_PWM_dutycycle(13, 127.5)
                                #sys.exit(0)
                        else:
                                pi.set_PWM_dutycycle(12, uk)
                                pi.set_PWM_dutycycle(13, Vd)

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


			mpc.header.stamp = rospy.get_rostime()
			mpc.desired.positions = [self.PT_x, self.PT_y, self.wpx_1, self.wpy_1, self.wpx_2, self.wpy_2]
			mpc.desired.effort = [self.r]

			mpc.actual.positions = [self.pos_x, self.pos_y, self.yaw]
			mpc.actual.velocities = [Vd, self.Vd]
			mpc.actual.effort = [uk, self.uk]

			mpc.error.positions = [self.distancia_PT, self.distancia]
			mpc.error.effort = [error1]

			self.pub1.publish(mpc)

                	#print("angle_goal",self.r)
                	#print("yaw",self.yaw)
			#print("speed", Vd)
                	#print("control_MPC_U", self.uk)
                	#print("error1:",error1)
			#print("xk", self.xk[2,0])
                	#print("self_yk", self.yk)
                	#print("wp1", self.wpx_1,self.wpy_1)
                	#print("wp2", self.wpx_2,self.wpy_2)
                	#print("rover", self.pos_x,self.pos_y)
                	#print("PT_X PT_Y", self.PT_x, self.PT_y)
                	#print("distancia_PT0", self.distancia_PT)
                	#print("distancia", self.distancia)
                	#print("ini", self.ini)
                	#print("contador", self.contador)
                	#print " "


	def main(self):
		rate = rospy.Rate(1/self.Ts)
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
		traceback_print_exc() #print "error_de_atributo: " + str(a)
		pi.set_PWM_dutycycle(13, 125.5)
		pi.set_PWM_dutycycle(12, 125.5)

	except pigpio.error:
                traceback.print_exc() #print "error_pigpio"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except IndexError, c:
                traceback.print_exc() #print "error_IndexdeLista: " + str(c)
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


