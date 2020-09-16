import serial
import time
import rospy
import numpy as np # se emplea esta para operar matrices
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

msg = Vector3Stamped()

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

        	self.pub = rospy.Publisher('/data_gps', Vector3Stamped, queue_size=10)


	def ubicacion(self):

		while not rospy.is_shutdown():
	    		if self.gps.isOpen():
            			self.line = self.gps.readline()
            			self.data = self.line.split(",")

				#if self.data[0] == "$GPGGA":
				#	print self.data

            			if self.data[0] == "$GPRMC":

                			Latitude = float(self.data[3])/100
                			Longitude = -float(self.data[5])/100

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

					msg.header.stamp = rospy.get_rostime()
					msg.vector.x = x
					msg.vector.y = y

					self.pub.publish(msg)

					#print x
					#print y
					#print " "


if __name__ == '__main__':
	try:
		rospy.init_node('ubicacion', anonymous=True, disable_signals=True)
		print "Nodo GPS creado"
    		cv = GPS()
		cv.ubicacion()

        except rospy.ROSInterruptException:
                pass
