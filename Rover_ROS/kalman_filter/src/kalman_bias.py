import time
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gps_common.msg import GPSFix

odom = Odometry()

class Kalman(object):

	def __init__(self):

		self.pub = rospy.Publisher('kalman_filter', Odometry, queue_size=10)
		self.ts = 0.05

		#IMU VARIABLES
		self.yaw = 0
		self.Acc_x = 0
		self.Acc_y = 0

		#GPS VARIABLES
		self.gps_x = 0
		self.gps_y = 0
		self.gps_0x = 0
		self.gps_0y = 0
		self.gps_distance = 0.0
		self.gps_vel = 0.0
		self.gps_vel2 = 0.0

		self.gpsx = 0.0
		self.gpsy = 0.0

		#ANGULAR VELOCITY YAW
		self.w = 0

		#KALMAN VARIABLES
		self.A = np.array([ [1,0,0,self.ts,0,-self.ts**2,0],
					[0,1,0,0,self.ts,0,-self.ts**2],
					[0,0,1,0,0,0,0],
					[0,0,0,1,0,0,0],
					[0,0,0,0,1,0,0],
					[0,0,0,0,0,1,0],
					[0,0,0,0,0,0,1]])

		self.Xkp = np.zeros([7,1])
		self.Xk = np.zeros([7,1])
		self.Xk_1 = np.zeros([7,1])

                self.B = np.array([ [0,self.ts**2,0],
                                        [0,0,self.ts**2],
                                        [self.ts,0,0],
                                        [0,self.ts,0],
                                        [0,0,self.ts],
					[0,0,0],
					[0,0,0]])

		self.U = np.zeros([3,1])
		self.U_1 = np.zeros([3,1])

		self.Pkp = np.zeros([7,7])
		self.Pk = np.zeros([7,7])
		self.Pk_1 = np.identity(7)*100

		self.Yk = np.zeros([6,1])
		self.Y = np.zeros([6,1])
		self.Y_a = np.zeros([6,1])

		self.H = np.array([ [0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0] ])

		self.C = np.array([ [1,0,0,0,0,0,0],
					[0,1,0,0,0,0,0],
					[0,0,1,0,0,0,0],
					[1,0,0,0,0,0,0],
					[0,1,0,0,0,0,0],
					[0,0,1,0,0,0,0]])

		self.K = np.zeros([7,6])
		self.K1 = np.zeros([7,6])
		self.K2 = np.zeros([7,7])

		self.I = np.identity(7)

		#MATRIZ DE COVARIANCE
		self.R = np.array([ [0.3,0,0,0,0,0],		# 1.1011
					[0,0.3,0,0,0,0],	# 0.7946
					[0,0,0.001,0,0,0],	# 0.0293
					[0,0,0,0.03,0,0],	# 0.0740
					[0,0,0,0,0.03,0],	# 0.1955
					[0,0,0,0,0,0.01] ])

                self.Q = np.array([ [0.2,0,0,0,0,0,0],
                                        [0,0.2,0,0,0,0,0],
                                        [0,0,0.1,0,0,0,0],
                                        [0,0,0,0.1,0,0,0],
                                        [0,0,0,0,0.1,0,0],
					[0,0,0,0,0,0.01,0],
					[0,0,0,0,0,0,0.01]])
		self.R1 = []

		#VARIABLES ODOMETRIA
		self.Pos_x = 0
		self.Pos_y = 0
		self.odom_yaw = 0

		self.count = 0

		#VARIABLES DEL MAG_YAW
		self.mag_yaw = 0.0
		self.mag_field_x = 0.0
		self.mag_field_y = 0.0

		self.samples = 0

		self.tiempo = 0.0

		rospy.Subscriber('/gps/data', GPSFix, self.gps_info)
		rospy.Subscriber('/odom', Odometry, self.odometria)
		rospy.Subscriber('/yaw', Imu, self.RB_imu)

	def gps_info(self,data):
		self.gps_distance = data.track
		self.gps_vel = data.speed*1/15.0

		gps_dx = self.gps_vel * np.cos(self.yaw)
		gps_dy = self.gps_vel * np.sin(self.yaw)

		self.gpsx = self.gpsx + gps_dx
		self.gpsy = self.gpsy + gps_dy

	def RB_imu(self,data):
		self.yaw = data.orientation.x
		self.Acc_x = data.linear_acceleration.x
		self.Acc_y = data.linear_acceleration.y
		self.w = data.angular_velocity.z

	def odometria(self,data):
                self.Pos_x = data.pose.pose.position.x
                self.Pos_y = data.pose.pose.position.y
		self.odom_yaw = data.pose.pose.position.z

	def filter(self):
		#ESTADO PREDICTOR
                self.U = np.array([ [self.w],
                                [self.Acc_x],
                                [self.Acc_y] ])

		self.Xkp = np.dot(self.A,self.Xk_1) + np.dot(self.B,self.U)
		self.Pkp = np.dot(np.dot(self.A,self.Pk_1),self.A.T) + self.Q

		theta_aux = self.Xkp[2,0] # ANGULO GENERADOR POR EL MODELO

		if self.Xkp[2,0] > np.pi:
			self.Xkp[2,0] -= 2.0*np.pi

		if self.Xkp[2,0] < -np.pi:
			self.Xkp[2,0] += 2.0*np.pi

		#NUEVA MEDICION
		self.Yk = np.dot(self.C,self.Xk_1)

		self.Y = np.array([ [self.gpsx],
				[self.gpsy],
				[self.yaw],
				[self.Pos_x],
				[self.Pos_y],
				[self.odom_yaw] ])

		#NUEVA MEDICION Y GANANCIA DE KALMAN
		self.K1 = np.dot(self.Pkp,self.C.T)
		self.K2 = np.dot(np.dot(self.C,self.Pkp),self.C.T) + self.R

		self.K = np.dot(self.K1,np.linalg.inv(self.K2))
		self.Y_a = self.Y - self.Yk

		# MEDICION Y SELECCION DEL ERROR MAS PEQUENO
		error1 = self.Y[2,0] - theta_aux
		error2 = self.Y[2,0] - self.Xkp[2,0]
		if error1 < -np.pi:
			self.Y_a[2,0] = self.Y_a[2,0] + 2*np.pi
		if error1 > np.pi:
			self.Y_a[2,0] = self.Y_a[2,0] - 2*np.pi

                error3 = self.Y[5,0] - theta_aux
		error4 = self.Y[5,0] - self.Xkp[2,0]
		if error3 < -np.pi:
			self.Y_a[5,0] = self.Y_a[5,0] + 2*np.pi
		if error3 > np.pi:
			self.Y_a[5,0] = self.Y_a[5,0] - 2*np.pi

		# ACTUALIZACION DEL VECTOR DE ESTADO
		self.Xk = self.Xkp + np.dot(self.K, self.Y_a)

		self.Pk1 = self.I - np.dot(self.K,self.C)
		self.Pk = np.dot(self.Pk1, self.Pkp)

		#INICIALIZACION
                if self.count <= 6:
			self.gps_0x = self.gps_x
			self.gps_0y = self.gps_y

			self.Xk[0,0] = self.gps_x - self.gps_0x
			self.Xk[1,0] = self.gps_y - self.gps_0y
			self.Xk[2,0] = self.yaw

		self.count += 1

		#ACTUALIZACION
		self.Xk_1 = self.Xk
		self.Pk_1 = self.Pk
		self.U_1 = np.array([ [self.w],
				[self.Acc_x],
				[self.Acc_y] ])

	def main(self):
		rate = rospy.Rate(1/self.ts)
                while not rospy.is_shutdown():
			self.filter()
                        odom.header.stamp = rospy.get_rostime()
			odom.header.frame_id = "KALMAN FILTER"
			odom.pose.pose.position.x = self.Xk[0,0]
			odom.pose.pose.position.y = self.Xk[1,0]
			odom.pose.pose.position.z = self.Xk[2,0]
			odom.twist.twist.angular.z = self.w
			odom.twist.twist.linear.x = self.gps_x - self.gps_0x
			odom.twist.twist.linear.y = self.gps_y - self.gps_0y

			self.pub.publish(odom)
			rate.sleep()

if __name__ == '__main__':
        try:
                rospy.init_node("kalman_filter")
                print "Nodo kalman_filter creado"
                cv = Kalman()
                cv.main()

        except rospy.ROSInterruptException:
                pass

        except KeyboardInterrupt:
                print "CLOSE"
