#! /usr/bin/env python
import serial
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class ubication(object):
        def __init__(self):
                self.pub = rospy.Publisher('/Position', Odometry, queue_size=10)

                self.enc_left = 0
                self.enc_right = 0
                self.enc_left_1 = 0
                self.enc_right_1 = 0
                self.enc_ts = 1.0/20.0
                self.Rpm_left = 0.0
                self.Rpm_right =0.0

                rospy.Subscriber('/enc_data', Odometry, self.read_enc)
                #rospy.Subscriber("informacion de la imu")
                #rospy.Subscriber("informacion del magnetometro")
                #rospy.Subscriber("informacion del gps")
                rospy.spin()

        def read_enc(self,data):
                odom = Odometry()
                self.enc_left = data.pose.covariance[5]
                enc_disL = self.enc_left - self.enc_left_1
                self.enc_left_1 = self.enc_left

                self.enc_right = data.pose.covariance[6]
                enc_disR = self.enc_right - self.enc_right_1
                self.enc_right_1 = self.enc_right


                # Angular Velocity and Lineal Velocity
                # 1500 son pulsos por RPM que tiene el encoder
                self.Rpm_left = (2*np.pi*enc_disL)/(1500.0*self.enc_ts)
                vel_left = self.Rpm_left*0.2

                self.Rpm_right = (2*np.pi*enc_disR)/(1500.0*self.enc_ts)
                vel_right = self.Rpm_right*0.2

                odom.twist.twist.angular.x = self.Rpm_left # Velocidad angular motor izquierdo
                odom.twist.twist.linear.x = vel_left # velocidad lineal motor izquierdo
                odom.twist.twist.angular.y = self.Rpm_right # Velocidad angular motor derecho
                odom.twist.twist.linear.y = vel_right # velocidad lineal motor derecho

                odom.header.stamp = rospy.get_rostime()
                self.pub.publish(odom)

if __name__=='__main__':
        try:
                rospy.init_node('Position',anonymous=True, disable_signals=True)
                print "Nodo Creado"
                cv = ubication()
        except rospy.ROSInterruptException:
                pass
