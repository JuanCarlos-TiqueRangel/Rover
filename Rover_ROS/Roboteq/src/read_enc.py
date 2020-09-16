#! /usr/bin/env python
import serial
import rospy
from sensor_msgs.msg import JointState

msg = JointState()

class roboteq(object):
        def __init__(self):
                # se define el puerto de lectura
                self.driver = serial.Serial(
                                port = "/dev/ttyACM0",
                                baudrate=115200)
                                #parity=serial.PARITY_ODD,
                                #stopbits=serial.STOPBITS_ONE,
                                #bytesize=serial.EIGHTBITS
                                #)

                self.pub = rospy.Publisher('/enc_data', JointState, queue_size=10)

		#Batery voltage
		self.battery = 0.0

		#Current motors
		self.I_left = 0.0
		self.I_right = 0.0

		#Encoders data
                self.enc_left = 0.0
                self.enc_right = 0.0

		self.load_roboteq = 0

		self.counter = 0.0

        def read_roboteq(self):

		self.driver.write("EELD\r")
		self.driver.write("!r\r")

		while not rospy.is_shutdown():
                	if self.driver.isOpen():
				odome = self.driver.readline()
				#print odome
				data = odome.split(",")
				#print len(data)

				#data_ = data.index("EELD")
				#print data_

				if 'EELD' not in data[0] and len(data) == 6:
					#Battery voltage
					self.battery  = float(data[0])

					#Motor current
					self.I_lefth = float(data[2])
					self.I_right = float(data[1])

					#encoder pulses
					self.enc_left = float(data[3])
					self.enc_right = float(data[4])

					#program counter
					self.counter = int(data[5])
					#print self.counter

				#ubicate in jointstate vector
				msg.velocity = [self.battery, self.I_left, self.I_right ,self.enc_left, self.enc_right, self.counter]
				msg.header.stamp = rospy.get_rostime()
                                self.pub.publish(msg)
				#self.rate.sleep()
                                #print msg

if __name__=='__main__':

       	try:
               	rospy.init_node('Odometria',anonymous=True, disable_signals=True)
		print "Nodo ENCODERS creado"
               	cv = roboteq()
		cv.read_roboteq()

       	except rospy.ROSInterruptException:
               	pass
