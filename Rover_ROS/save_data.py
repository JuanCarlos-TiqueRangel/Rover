#! /usr/bin/env python
import serial
import rospy
import message_filters
import numpy as np

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

class adq_datos(object):
        def __init__(self):
                self.datos = [0,0,0,0,0,0,0,0,0,0,0,0]

                #self.pwm = message_filters.Subscriber('/PWM', JointState)
                self.encoder = message_filters.Subscriber('/enc_data', Odometry)
                self.gps = message_filters.Subscriber('/gps', Odometry)
                self.imu_acc = message_filters.Subscriber('/mti/sensor/imu_free', Imu)
                self.imu_mag = message_filters.Subscriber('/mti/sensor/magnetic', Vector3Stamped)

                self.ts = message_filters.ApproximateTimeSynchronizer([self.encoder, self.gps, self.imu_acc, self.imu_mag], queue_size= 5, slop=10)
                self.ts.registerCallback(self.synchronize_data)
                rospy.spin()

        def synchronize_data(self, encoder, gps, imu_acc, imu_mag):
                self.datos[0] =0 #pwm.velocity[0]
                self.datos[1] =0 #pwm.velocity[1]
                self.datos[2] = encoder.pose.covariance[5]
                self.datos[3] = encoder.pose.covariance[6]
                self.datos[4] = gps.pose.pose.position.x
                self.datos[5] = gps.pose.pose.position.y
                self.datos[6] = imu_acc.linear_acceleration.x
                self.datos[7] = imu_acc.linear_acceleration.y
                self.datos[8] = imu_acc.linear_acceleration.z
                self.datos[9] = imu_mag.vector.x
                self.datos[10] = imu_mag.vector.y
                self.datos[11] = imu_mag.vector.z

                f = open('file.txt','a')

                f.write(format(self.datos[0])+'\t'+format(self.datos[1])+format(self.datos[2])+'\t'+format(self.datos[3])+'\t'+format(self.datos[4])+'\t'+format(self.datos[5])+'\t'+format(self.datos[6])+'\t'+format(self.datos[7])+'\t'+format(self.datos[8])+'\t'+format(self.datos[9])+'\t'+format(self.datos[10])+'\t'+format(self.datos[11])+'\n')
                f.close()

                print self.datos


if __name__=='__main__':
        try:
                rospy.init_node('save_data',anonymous=True, disable_signals=True)
                print "Nodo Creado"
                cv = adq_datos()
        except rospy.ROSInterruptException:
                pass

