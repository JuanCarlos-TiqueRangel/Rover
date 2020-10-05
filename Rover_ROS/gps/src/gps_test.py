import serial
import time
import rospy
import numpy as np # se emplea esta para operar matrices

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from gps_common.msg import GPSStatus, GPSFix

msg = Vector3Stamped()
gps = GPSFix()
gps2 = GPSStatus()

#gps = serial.Serial("/dev/tty_gps", baudrate = 9600)

class GPS(object):

    	def __init__(self):
        	self.gps = serial.Serial("/dev/tty_gps", baudrate = 9600)
        	self.line = 0
        	self.data = 0

        	#Geometria del elipsoide
        	self.a = 6378137.0 # radius of the earth in metters
        	self.b = 6356752.31414

        	self.e1 = 6.6943800229e-3
        	self.e2 = 6.73949677548e-3
        	#radio polar de curvatura = c
        	self.c = 6399593.626
        	self.Long_rad = 0
        	self.Lat_rad = 0

		self.satelites = 0
		self.altitude = 0.0
		self.distance = 0.0
		self.MeasureCounting = 0
		self.time_utc = 0.0
		self.speed_m_s = 0.0

        	self.pub = rospy.Publisher('/gps/utm', Vector3Stamped, queue_size=10)
		self.pub1 = rospy.Publisher('/gps/data', GPSFix, queue_size=10)

	def GpsTimeSeconds(self, Time_Gps):
		H = float(Time_Gps[0:2])
		M = float(Time_Gps[2:4])
		S = float(Time_Gps[4:9])

		Time_seconds = H*3600 + M*60 + S
		return Time_seconds

	def ubicacion(self):

		while not rospy.is_shutdown():
	    		if self.gps.isOpen():
            			self.line = self.gps.readline()
            			self.data = self.line.split(",")

				if self.data[0] == "$GPGGA":
					#print self.data
					self.satelites = float(self.data[7])
					self.altitude = float(self.data[9])

            			if self.data[0] == "$GPRMC":

                			Latitude = float(self.data[3])/100.0
                			Longitude = -float(self.data[5])/100.0

					## NUEVO ============================================
					self.speed_m_s = float(self.data[7]) * 0.514444
					self.time_utc = float(self.data[1])

					if self.speed_m_s < 0.5:
						self.speed_m_s = 0.0

					self.MeasureCounting += 1
					if self.MeasureCounting == 1:
						t_0 = self.GpsTimeSeconds(self.data[1])
					else:
						t_1 = self.GpsTimeSeconds(self.data[1])
						self.distance = (t_1 - t_0) * self.speed_m_s
						t_0 = t_1
					#FIN DE LO NUEVO =======================================

                			self.Long_rad = Longitude * (np.pi/180.0)
                			self.Lat_rad = Latitude * (np.pi/180.0)

                			#huso to use in Colombia
                			huso = 18

                			#Get the central meridian of huso = lamnda0
                			lamnda0 = (huso * 6.0 - 183.0)*(np.pi/180)

                			#Determination of anglular distance that exist between
                			#point Longitude and central meridian of huso
                			delta_lambda0 = self.Long_rad - lamnda0

                			#Coticchia-Surace ecuations for the direct problem
                			#Switch geographics to UTM
                			#Estimation of parameters

                			A = np.cos(self.Lat_rad) * np.sin(delta_lambda0)
                			xi = 0.5 * np.log((1+A)/(1-A))
                			eta = np.arctan(np.tan(self.Lat_rad)/np.cos(delta_lambda0)) - self.Lat_rad
                			nu = (self.c*0.9996)/np.sqrt((1 + self.e2*np.cos(self.Lat_rad)*np.cos(self.Lat_rad)))
                			zeta = (self.e2/2)*(xi*xi)*(np.cos(self.Lat_rad)*np.cos(self.Lat_rad))
                			A1 = np.sin(2.0*self.Lat_rad)
                			A2 = A1 * (np.cos(self.Lat_rad)*np.cos(self.Lat_rad))
                			J2 = self.Lat_rad + A1/2.0
                			J4 = (3*J2 + A2)/4
                			J6 = (5*J4 + A2 * (np.cos(self.Lat_rad)*np.cos(self.Lat_rad)))/3
                			alpha2 = (3.0/4.0)*(self.e2)
                			beta = (5.0/3.0)*(alpha2*alpha2)
                			gamma = (35.0/27.0)*(np.power(alpha2,2))
                			B_phi = 0.9996 * self.c * (self.Lat_rad - alpha2 * J2 + beta * J4 - gamma * J6)

                			x = xi*nu*(1+zeta/3.0)+500000.0
                			y = eta*nu*(1+zeta)+B_phi

					# PUBLISH DATA
					gps.header.stamp = rospy.get_rostime()
					gps.header.frame_id = "GPS DATA"
					gps.status.header.frame_id = "/GPS_DATA"
					gps.latitude = Latitude
					gps.longitude = Longitude
					gps.altitude = self.altitude
					gps.speed = self.speed_m_s
					gps.status.satellites_used = self.satelites
					gps.time = self.time_utc
					gps.track = self.distance

					msg.header.stamp = rospy.get_rostime()
					msg.vector.x = x
					msg.vector.y = y
					msg.vector.z = self.satelites

					self.pub.publish(msg)
					self.pub1.publish(gps)
					#print(gps)

					#print x
					#print y
					#print " "


if __name__ == '__main__':
	try:
		rospy.init_node("GPS_DATA")
		print "Nodo GPS creado"
    		cv = GPS()
		cv.ubicacion()

        except rospy.ROSInterruptException:
                pass
