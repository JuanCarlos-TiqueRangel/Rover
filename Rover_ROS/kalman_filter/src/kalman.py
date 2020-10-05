import time
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

odom = Odometry()

class Kalman(object):

	def __init__(self):

		self.pub = rospy.Publisher('kalman_filter', Odometry, queue_size=10)
		self.ts = 0.1

		#IMU VARIABLES
		self.yaw = 0
		self.Acc_x = 0
		self.Acc_y = 0

		#GPS VARIABLES
		self.gps_x = 0
		self.gps_y = 0
		self.gps_0x = 0
		self.gps_0y = 0

		#ANGULAR VELOCITY YAW
		self.w = 0

		#KALMAN VARIABLES
		self.A = np.array([ [1,0,0,self.ts,0],
					[0,1,0,0,self.ts],
					[0,0,1,0,0],
					[0,0,0,1,0],
					[0,0,0,0,1] ])

		self.Xkp = np.zeros([5,1])
		self.Xk = np.zeros([5,1])
		self.Xk_1 = np.zeros([5,1])

                self.B = np.array([ [0,0,0],
                                        [0,0,0],
                                        [self.ts,0,0],
                                        [0,self.ts,0],
                                        [0,0,self.ts] ])

		self.U = np.zeros([3,1])

		self.Pkp = np.zeros([5,5])
		self.Pk = np.zeros([5,5])
		#self.Pk_1 = np.zeros([5,5])
		self.Pk_1 = np.identity(5)*100

		self.Yk = np.zeros([6,1])
		self.Y = np.zeros([6,1])
		self.Y_a = np.zeros([6,1])

		self.H = np.array([ [0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0],
				[0,0,0,0,0] ])

		self.C = np.array([ [1,0,0,0,0],
					[0,1,0,0,0],
					[0,0,1,0,0],
					[1,0,0,0,0],
					[0,1,0,0,0],
					[0,0,1,0,0] ])

		self.K = np.zeros([5,5])
		self.K1 = np.zeros([5,6])
		self.K2 = np.zeros([6,6])

		self.I = np.identity(5)

		#MATRIZ DE COVARIANCE
		self.R = np.array([ [1.5,0,0,0,0,0],
					[0,1.5,0,0,0,0],
					[0,0,0.0263,0,0,0],
					[0,0,0,0.5,0,0],
					[0,0,0,0,0.5,0],
					[0,0,0,0,0,0] ])

                self.Q = np.array([ [0.5,0,0,0,0],
                                        [0,0.5,0,0,0],
                                        [0,0,0.01,0,0],
                                        [0,0,0,0.5,0],
                                        [0,0,0,0,0.5] ])
		self.R1 = []

		#VARIABLES ODOMETRIA
		self.Pos_x = 0
		self.Pos_y = 0
		self.odom_yaw = 0

		self.count = 0

		#VARIABLES DEL MAG_YAW
		self.mag_yaw = 0

		self.samples = 0

		rospy.Subscriber('/gps', Vector3Stamped, self.gps_data)
		rospy.Subscriber('/odom', Odometry, self.odometria)
		rospy.Subscriber('/yaw', Imu, self.RB_imu)

	def gps_data(self,data):
		self.gps_x = data.vector.x
		self.gps_y = data.vector.y


	def RB_imu(self,data):
		self.yaw = data.orientation.x
		self.mag_yaw = data.orientation.y

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

		#NUEVA MEDICION
		self.Yk = np.dot(self.C,self.Xk_1)

		self.Y = np.array([ [self.gps_x - self.gps_0x],
                		[self.gps_y - self.gps_0y],
                		[self.mag_yaw],
                		[self.Pos_x],
                		[self.Pos_y],
                		[self.odom_yaw] ])

		#NUEVA MEDICION Y GANANCIA DE KALMAN
		self.K1 = np.dot(self.Pkp,self.C.T)
		self.K2 = np.dot(np.dot(self.C,self.Pkp),self.C.T) + self.R

		self.K = np.dot(self.K1,np.linalg.inv(self.K2))

		self.Y_a = self.Y - self.Yk
		self.Xk = self.Xkp + np.dot(self.K, self.Y_a)

		self.Pk1 = self.I - np.dot(self.K,self.C)
		self.Pk = np.dot(self.Pk1, self.Pkp)

		#INICIALIZACION
                if self.count <= 6:
			self.gps_0x = self.gps_x
			self.gps_0y = self.gps_y

			self.Xk[0,0] = self.gps_x - self.gps_0x
			self.Xk[1,0] = self.gps_y - self.gps_0y
			self.Xk[2,0] = self.mag_yaw

		self.count += 1

		while self.Xk[2,0] > np.pi:
			self.Xk[2,0] -= 2.0*np.pi

		while self.Xk[2,0] < -np.pi:
			self.Xk[2,0] += 2.0*np.pi

		#ACTUALIZACION
		self.Xk_1 = self.Xk
		self.Pk_1 = self.Pk

		#print(self.Xk)
		#print " "

	def main(self):

                rate = rospy.Rate(20)
                while not rospy.is_shutdown():

			self.filter()

                        odom.header.stamp = rospy.get_rostime()

			odom.header.frame_id = "KALMAN FILTER"
			odom.pose.pose.position.x = self.Xk[0,0]
			odom.pose.pose.position.y = self.Xk[1,0]
			odom.pose.pose.position.z = self.Xk[2,0]
			odom.twist.twist.angular.z = self.w

			self.pub.publish(odom)

			#print(self.odom_yaw)
			#print " "

			rate.sleep()

if __name__ == '__main__':
        try:

                rospy.init_node("Kalman_Filter")
                print "Nodo kalman_filter creado"
                cv = Kalman()
                cv.main()

        except rospy.ROSInterruptException:
                pass

        except KeyboardInterrupt:
                print "CLOSE"
