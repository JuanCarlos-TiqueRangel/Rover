import rospy
import smbus
from time import sleep
import math
import numpy as np

#some MPU6050 Registers and their Address
Register_A     = 0		#Address of Configuration register A
Register_B     = 0x01		#Address of configuration register B
Register_mode  = 0x02		#Address of mode register

X_axis_H    = 0x03		#Address of X-axis MSB data register
Z_axis_H    = 0x05		#Address of Z-axis MSB data register
Y_axis_H    = 0x07		#Address of Y-axis MSB data register
declination = -0.00669		#define declination angle of location where measurement going to be $
pi          = 3.14159265359	#define pi value

bus = smbus.SMBus(1)
Device_Address = 0x1e

class magnetometer(object):

        def __init__(self):

                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

                self.xmin = 0.0
                self.ymin = 0.0
                self.xmax = 0.0
                self.ymax = 0.0

                self.yaw = 0.0

        #def magnetometer_init(self):
		#write to Configuration Register A
		bus.write_byte_data(Device_Address, Register_A, 0x70)

		#Write to Configuration Register B for gain
		bus.write_byte_data(Device_Address, Register_B, 0xa0)

		#Write to mode Register for selecting mode
		bus.write_byte_data(Device_Address, Register_mode, 0x00)


        def read_raw_data(self, addr):
                #Read raw 16-bit value
                high = bus.read_byte_data(Device_Address, addr)
                low = bus.read_byte_data(Device_Address, addr+1)

                #concatenate higher and lower value
                value = ((high << 8) | low)

                #to get signed value from module
                if(value > 32768):
                        value = value - 65536
                return value

        def main(self):

                rate = rospy.Rate(20)
                while not rospy.is_shutdown():
                        #Read Accelerometer raw value
                        x = self.read_raw_data(X_axis_H)
                        z = self.read_raw_data(Z_axis_H)
                        y = self.read_raw_data(Y_axis_H)

                        if x>self.xmax: self.xmax = x
                        if y>self.ymax: self.ymax = y

                        if x<self.xmin: self.xmin = x
                        if y<self.ymin: self.ymin = y

                        xsf = (self.ymax - self.ymin)/(self.xmax - self.xmin)
                        ysf = (self.xmax - self.xmin)/(self.ymax - self.ymin)

                        xoff = ((self.xmax - self.xmin)/2 - self.xmax)*xsf
                        yoff = ((self.ymax - self.ymin)/2 - self.ymax)*ysf

                        self.x = xsf*x + xoff
                        self.y = ysf*y + yoff

                        heading = np.arctan2(self.y, self.x) #+ declination

                        #Due to declination check for >360 degree
                        if(heading > pi):
                                heading = heading - 2*pi

                        #check for sign
                        if(heading < -pi):
                                heading = heading + 2*pi

                        #convert into angle
                        heading_angle = heading * 180/pi

                        print ("angle",heading_angle)
                        rate.sleep()



if __name__=='__main__':
        try:
                rospy.init_node("magnetic")
                print "nodo mag creado"
                cv = magnetometer()
                cv.main()

        except rospy.ROSInterruptException:
                pass

