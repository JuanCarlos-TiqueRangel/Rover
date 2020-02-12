import serial
import time
import rospy
import numpy as np # se emplea esta para operar matrices
from nav_msgs.msg import Odometry

msg = Odometry()

class GPS(object):

        def __init__(self):
                self.MeasureCounting = 0

                self.distance = 0

                self.gps = serial.Serial("/dev/tty_gps", baudrate = 9600)
                self.pub = rospy.Publisher('/gps_dis', Odometry, queue_size=10)
                self.ubicacion()

        def GpsTimeSeconds(self, Time_Gps):
                H = float(Time_Gps[0:2])
                M = float(Time_Gps[2:4])
                S = float(Time_Gps[4:9])

                Time_seconds = H*3600 + M*60 + S
                #print H
                return Time_seconds

        def ubicacion(self):
                if self.gps.isOpen():
                        k =0
                        while True:
                                self.line = self.gps.readline()
                                data = self.line.split(",")
                                #print data

                                if (data[0] == "$GPRMC") and (data[2]=='A'):
                                        self.MeasureCounting = self.MeasureCounting + 1
                                        if self.MeasureCounting == 1:
                                                t_0 = self.GpsTimeSeconds(data[1])
                                        else:
                                                t_1 = self.GpsTimeSeconds(data[1])
                                                vel_m_s = float(data[7]) * 0.514444
                                                if vel_m_s < 0.5:
                                                        vel_m_s = 0

                                                self.distance = (t_1 - t_0) * vel_m_s
                                                t_0 = t_1

                                                #print self.distance

                                msg.header.stamp = rospy.get_rostime()
                                msg.pose.pose.position.x = self.distance
                                self.pub.publish(msg)


if __name__ == '__main__':
        try:
                rospy.init_node('distancia', anonymous=True, disable_signals=True)
                print "Nodo GPS creado"
                cv = GPS()
        except rospy.ROSInterruptException:
                pass
