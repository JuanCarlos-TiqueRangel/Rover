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

		#self.driver.write("EELD\r")

                self.pub = rospy.Publisher('/enc_data', JointState, queue_size=10)

		#Batery voltage
		self.voltage_battery = 0.0

		#Temperature motors
		self.T_left = 0.0
		self.T_right = 0.0

		#Current motors
		self.I_left = 0.0
		self.I_right = 0.0

		#Encoders data
                self.enc_left = 0.0
                self.enc_right = 0.0


		self.counter = 0.0
		self.Ts_roboteq = 0.0

		#self.rate = rospy.Rate(10)
                #self.read_roboteq()

        def read_roboteq(self):
		while not rospy.is_shutdown():
                	if self.driver.isOpen():
				self.driver.write("EELD\r")
				odome = self.driver.readline()
				data = odome.split(",")
				#print odome
				if len(data) == 8:
					#Battery Voltage
					battery = data[0]
					b_battery = battery.split("A")
					self.voltage_battery = b_battery[1]

					#Motor temperature
					self.T_left = data[2]
					self.T_right = data[1]

                                	#Motor current
                                	self.I_left = data[4]
                                	self.I_right = data[3]

					#pulses encoder
					self.enc_left = float(data[6])
					self.enc_right = float(data[5])

					#Counter
                                	counter = data[7]
                                	c_counter = counter.split("A")
                                	self.counter = float(c_counter[0])

				#ubicate in jointstate vector
				msg.velocity = [self.enc_left, self.enc_right, self.counter]
				msg.header.stamp = rospy.get_rostime()
                                self.pub.publish(msg)
				#self.rate.sleep()
                                	#print self.enc_right

if __name__=='__main__':

       	try:
               	rospy.init_node('Odometria',anonymous=True, disable_signals=True)
		print "Nodo ENCODERS creado"
               	cv = roboteq()
		cv.read_roboteq()
       	except rospy.ROSInterruptException:
               	pass

