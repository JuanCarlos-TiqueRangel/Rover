import serial
import time
import rospy
import numpy as np # se emplea esta para operar matrices

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from gps_common.msg import GPSStatus, GPSFix

msg = Odometry()
gps = GPSFix()
gps2 = GPSStatus()

gps_ = serial.Serial("/dev/tty_gps", baudrate = 115200)

class GPS(object):

    	def __init__(self):
		#self.gps = serial.Serial("/dev/tty_gps", baudrate = 115200)

        	#Geometria del elipsoide
		self.a = 6378137.0 # radius of the earth in metters
		self.b = 6356752.31414

		self.e1 = 6.6943800229e-3
		self.e2 = 6.73949677548e-3
		#radio polar de curvatura = c
		self.c = 6399593.626

		self.satelites = 0
		self.altitude = 0.0
		self.distance = 0.0
		self.MeasureCounting = 0
		self.time_utc = 0.0
		self.speed_m_s = 0.0

		self.Longitude = 0.0
		self.Latitude = 0.0
		self.x = 0.0
		self.y = 0.0

		self.ds = 0.0
		self.flag_time = True
		self.t_1 = 0.0
		self.t_0 = 0.0

		self.pub = rospy.Publisher('/gps/utm', Odometry, queue_size=10)
		self.pub1 = rospy.Publisher('/gps/data', GPSFix, queue_size=10)

	def GpsTimeSeconds(self, Time_Gps):
		H = float(Time_Gps[0:2])
		M = float(Time_Gps[2:4])
		S = float(Time_Gps[4:9])

		Time_seconds = H*3600 + M*60 + S
		return Time_seconds

	def gps_read(self, data):

		line = data.readline()
		data = line.split(",")

		if data[0] == "$GPGGA":
			self.satelites = float(data[7])
			self.altitude = float(data[9])

		if (data[0] == "$GPRMC") and (data[2]=='A'):

			self.Latitude = int(float(data[3])/100.0)
			self.Latitude = self.Latitude + (float(data[3])%100.0)/60.0
			if data[4] == 'N':
				self.Latitude = self.Latitude*1.0
			elif data[4] == 'S':
				self.Latitude = self.Latitude*-1.0

			self.Longitude = int(float(data[5])/100.0)
			self.Longitude = self.Longitude + (float(data[5])%100.0)/60.0
			if data[6] == 'E':
				self.Longitude = self.Longitude*1.0
			elif data[6] == 'W':
				self.Longitude = self.Longitude*-1.0

			#DISTANCIA A PARTIR DE LA VELOCIDAD
			#self.speed_m_s = float(data[7]) * 0.514444
			self.time_utc = float(data[1])

			#if self.speed_m_s < 0.5:
			#	self.speed_m_s = 0.0

			self.MeasureCounting = self.MeasureCounting + 1
			if self.MeasureCounting == 1:
			#if self.flag_time == True:
				self.t_0 = self.GpsTimeSeconds(data[1])
				self.flag_time = False
			else:
				self.t_1 = self.GpsTimeSeconds(data[1])
				self.speed_m_s = float(data[7]) * 0.514444
				if self.speed_m_s < 0.5:
					self.speed_m_s = 0.0

				self.distance = (self.t_1 - self.t_0) * self.speed_m_s
				self.t_0 = self.t_1

		#self.ds += self.speed_m_s * 1


	def grades_to_utm(self, latitude, longitude):

		Long_rad = longitude * (np.pi/180.0)
		Lat_rad = latitude * (np.pi/180.0)

		#huso to use in Colombia
		huso = 18

		#Get the central meridian of huso = lamnda0
		lamnda0 = (huso * 6.0 - 183.0)*(np.pi/180)

		#Determination of anglular distance that exist between
		#point Longitude and central meridian of huso
		delta_lambda0 = Long_rad - lamnda0

		#Coticchia-Surace ecuations for the direct problem
		#Switch geographics to UTM
		#Estimation of parameters

		A = np.cos(Lat_rad) * np.sin(delta_lambda0)
		xi = 0.5 * np.log((1+A)/(1-A))
		eta = np.arctan(np.tan(Lat_rad)/np.cos(delta_lambda0)) - Lat_rad
		nu = (self.c*0.9996)/np.sqrt((1 + self.e2*np.cos(Lat_rad)*np.cos(Lat_rad)))
		zeta = (self.e2/2)*(xi*xi)*(np.cos(Lat_rad)*np.cos(Lat_rad))
		A1 = np.sin(2.0*Lat_rad)
		A2 = A1 * (np.cos(Lat_rad)*np.cos(Lat_rad))
		J2 = Lat_rad + A1/2.0
		J4 = (3*J2 + A2)/4
		J6 = (5*J4 + A2 * (np.cos(Lat_rad)*np.cos(Lat_rad)))/3
		alpha2 = (3.0/4.0)*(self.e2)
		beta = (5.0/3.0)*(alpha2*alpha2)
		gamma = (35.0/27.0)*(np.power(alpha2,3))
		B_phi = 0.9996 * self.c * (Lat_rad - alpha2 * J2 + beta * J4 - gamma * J6)

		self.x = xi*nu*(1+zeta/3.0)+500000.0
		self.y = eta*nu*(1+zeta)+B_phi

	def ubicacion(self):

		if gps_.isOpen():
			rate = rospy.Rate(15)
			while not rospy.is_shutdown():
				try:
					self.gps_read(gps_)
					self.grades_to_utm(self.Latitude, self.Longitude)
				except:
					pass

				# PUBLISH DATA
				gps.header.stamp = rospy.get_rostime()
				gps.header.frame_id = "GPS DATA"
				gps.status.header.frame_id = "/GPS_DATA"
				gps.latitude = self.Latitude
				gps.longitude = self.Longitude
				gps.altitude = self.altitude
				gps.speed = self.speed_m_s
				gps.status.satellites_used = self.satelites
				gps.time = self.time_utc
				gps.track = self.distance

				#print self.time_utc

				msg.header.stamp = rospy.get_rostime()
				msg.header.frame_id = " UTM_COORDINATE "
				msg.pose.pose.position.x = self.x
				msg.pose.pose.position.y = self.y
				msg.pose.pose.position.z = self.altitude
				msg.twist.twist.linear.x = self.speed_m_s

				self.pub.publish(msg)
				self.pub1.publish(gps)
				rate.sleep()

				#print self.distance

				#print(gps)

				#print self.Latitude
				#print self.Longitude
				#print " "


if __name__ == '__main__':
	try:
		rospy.init_node("GPS_DATA")
		print "Nodo GPS creado"
    		cv = GPS()
		cv.ubicacion()

        except rospy.ROSInterruptException:
                pass
