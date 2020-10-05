import rospy
import numpy as np
import time
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Vector3Stamped
from sensor_msgs.msg import Imu

msg = Twist()
goal = Point()

class track(object):

        def __init__(self):

                self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)

                rospy.Subscriber("/kalman_filter", Vector3Stamped, self.trajectory)

                self.speed = 0
                self.pos_x = 0
                self.pos_y = 0
                self.yaw = 0
                self.yaw_1 = 0
                self.angle_to_goal = 0.0
		self.Ts = 0.1

		# VARIABLES CONTROLADOR
		self.delta_w = 0.0
		self.w = 0.0
		self.w_1 = 0.0

		self.delta_pos = 0.0
		self.acc = 0.0
		self.acc_1 = 0.0
		self.delta_acc = 0.0
		self.tiron = 0.0

		self.yk = 0.0
		self.yk_1 = 0.0
		# FIN DE VARIABLES CONTROLADOR

		# === VARIABLES DEL GENERADOR DE TRAYECTORIA ====
                #self.goalx = [0,-10,-10,0,0,-10,0]
                #self.goaly = [0,0,5,5,10,10,0]

		self.goalx = [0,10,10,0,0,10,0]
		self.goaly = [0,0,5,5,10,10,0]
                self.ini = 0
                self.ini_old = 0
		self.contador = 0
		self.wpx_1 = 0
		self.wpy_1 = 0
		self.wpx_2 = 0
		self.wpy_2 = 0
		self.alpha = 0
		self.distancia = 0
		self.PT_x = 0
		self.PT_y = 0
		self.distancia_PT = 0
		## ===============================================

		##==================================================================================##
                #			VARIABLES MODELO CONTROLADOR 				     #
		##==================================================================================##
                self.A = np.array([[0.4534, -2.705, 0],
                                [0.07243, 0.8477, 0],
                                [0.00408, 0.09464, 1]])

                self.B = np.array([ [0.07243],[0.00408],[0.0001437] ])

                self.C = np.array([ [0,0,65.7] ])

                self.Np = 24
                self.Nc = 25

		##================== MODELO EXTENDIDO ================================================
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
                self.r = 0              # SET POINT

                self.u=0; # u(k-1) =0
                self.y=0;
                #u1 = np.zeros(N_sim)
                #y1 = np.zeros(N_sim)

                self.h = self.C_e
                self.F = np.dot(self.C_e,self.A_e)

                for kk in range(1,self.Np):
                        self.h = np.vstack((self.h,np.dot(self.h[kk-1,:],self.A_e)))
                        self.F = np.vstack((self.F,np.dot(self.F[kk-1,:],self.A_e)))

                self.v = np.dot(self.h,self.B_e)
                self.Phi = np.zeros([self.Np,self.Nc]); #declare the dimension of Phi
                self.Phi[:,0] = self.v[:,0]

                for i in range(1,self.Nc):
                        self.Phi[i:,i] = self.v[0:self.Np-i:,0]

                self.BarRs = np.ones([self.Np,1])
                self.Phi_Phi = np.dot(self.Phi.T,self.Phi)
                self.Phi_F = np.dot(self.Phi.T,self.F)
                self.Phi_R = np.dot(self.Phi.T,self.BarRs)
                self.xk = np.zeros(self.B_e.shape)
                self.xk_1 = np.zeros(self.B_e.shape)
		self.u1 = 0

		## =====================VARIABLES DE LA ACCION DE CONTROL ==========================================
                self.Ky = np.dot(np.linalg.inv(self.Phi_Phi+2.5*np.identity(self.Nc)), self.Phi_R)
                self.Ky = self.Ky[0,0]

                self.K_mpc = np.dot(np.linalg.inv(self.Phi_Phi+2.5*np.identity(self.Nc)), self.Phi_F)
                self.K_mpc = np.array(self.K_mpc[0,:], ndmin=2)

		##================================================================================================##
                # 			FIN DE LAS VARIALBES DEL CONTROLADOR					   #
		##================================================================================================##
		self.Ui_1 = 0
		self.error_1 = 0
		self.Ts = 0.1
		self.comp = 0

        def trajectory(self, data):

		##============ UBICACION Y ORIENTACION ====================
                self.pos_x = data.vector.x
                self.pos_y = data.vector.y
		self.yaw = data.vector.z
		## ========================================================

                # WAY POINTS
		self.wpx_1 = self.goalx[self.ini]
		self.wpy_1 = self.goaly[self.ini]
		self.wpx_2 = self.goalx[self.ini+1]
		self.wpy_2 = self.goaly[self.ini+1]

		# VECTOR DISTANCIA
		delta_wpx = self.wpx_2 - self.wpx_1
		delta_wpy = self.wpy_2 - self.wpy_1

                #delta_wpx = self.wpx_2 - self.pos_x
                #delta_wpy = self.wpy_2 - self.pos_y
		self.alpha = np.arctan2(delta_wpy, delta_wpx)

		# =========================== INTERPUNTO ======================================
                self.PT_x = self.Ts*0.2*self.contador*np.cos(self.alpha) + self.wpx_1
                self.PT_y = self.Ts*0.2*self.contador*np.sin(self.alpha) + self.wpy_1
		## ============================================================================


                # ANGULO DE CONTROL
                self.angle_to_goal = np.arctan2(self.PT_y - self.pos_y, self.PT_x - self.pos_x)

                ## DISTANCIA ==============================================
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


        def predictive_control(self):

		## ============================== IMU DATA =====================================================
                self.delta_pos = self.yaw - self.yaw_1
                self.delta_w = self.w - self.w_1

                self.acc = self.delta_w/self.Ts
                self.delta_acc = self.acc - self.acc_1
		##==============================================================================================

		self.r = self.angle_to_goal

                if self.r > 0:
                        self.comp = self.r - 2*np.pi
                else:
                        self.comp = self.r + 2*np.pi

		error1 = self.angle_to_goal - self.yaw
		error2 = self.angle_to_goal + self.comp

                if abs(error1) > abs(error2):
                        error = error2

		else:
			error = error1

		##=============================================================================================##
		#				CONTROL	PREDICTIVO						#
		##=============================================================================================##
                self.deltaU = self.Ky*self.r - np.dot(self.K_mpc, self.xk)
                self.u = self.deltaU + self.u

                self.yk = self.yaw
		self.xk = np.array([ [self.delta_pos],[self.delta_w],[self.delta_acc],[self.yk - self.yk_1] ])
		## ============================================================================================


                ## ===================================== PID ==================================================
                Ts = 0.1
                kp = 1.1
                ki = 0.0
                kd = 0.5

		Up = kp*error
		Ui = ki*(error) + self.Ui_1*Ts
		Ud = (kd/Ts)*(error-self.error_1)
		U = Up+Ui+Ud
		#self.error_1 = error
		#self.Ui_1 = Ui
		##=============================================================================================

		U_w = self.u/255

		if U_w  > 10.0:
			U_w = 10.0

		if U_w < -10.0:
			U_w = -10.0

		if self.ini + 2 > len(self.goalx):
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			sys.exit(0)
		else:
			msg.linear.x = 0.2
			msg.angular.z = U

		self.pub.publish(msg)

		#ACTUALIZACION DE VARIABLES
		self.yk_1 = self.yk
                self.w_1 = self.w
                self.yaw_1 = self.yaw
                self.acc_1 = self.acc

		# V-PID
		self.error_1 = error
		self.Ui_1 = Ui

		#print("angle_goal", self.r)
		#print("self.yaw", self.yaw)
		#print("control", U)
		#print("delta_pos", self.delta_pos)
		#print("delta_w",self.delta_w)
		#print("delta_acc", self.delta_acc)
                #print("wp1", self.wpx_1,self.wpy_1)
                #print("rover", self.pos_x,self.pos_y)
                #print("PT_X PT_Y", self.PT_x, self.PT_y)
		#print("Kmpc*x ",np.dot(self.K_mpc,self.xk))
		#print("ky*ref", self.Ky*self.angle_to_goal*180/np.pi)
		#print("KY",self.Ky)
		#print("K_MPC", self.K_mpc)
		#print " "

		print("angle_goal",self.r)
		print("yaw",self.yaw)
		print("angulo_coordenada", self.alpha)
		print("control", U)
		#print("error1:",error1)
		#print("error2:",error2)
		#print("error:",error)
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
                rate = rospy.Rate(10)
                while not rospy.is_shutdown():
                        	self.predictive_control()
                        	rate.sleep()

if __name__=='__main__':
        try:
                rospy.init_node('main',anonymous=True, disable_signals=True)
                cv = track()
                #print "Nodo PWM creado"
                cv.main()

        except rospy.ROSInterruptException:
                print " CLOSE "

	except IndexError:
		msg.linear.x = 0.0
		msg.angular.z = 0.0
