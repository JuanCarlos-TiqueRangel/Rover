import rospy
import time
import serial
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import String

class MOTORES(object):

    def __init__(self):
        self.cuenta = 0
        self.A = 0
        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
            )

        self.pub = rospy.Publisher('/mover_motor', String, queue_size=10)
        self.movimiento()
        #rospy.init_node('/motor',anonymous=True)


    def movimiento(self):
        if self.ser.isOpen():
            for i in range(0,1000,1):
                self.A = "!M -"+ str(i)
                self.ser.write(self.A)
                self.ser.write("\r")
                self.pwm = "pwm: " + str(i)
                self.pub.publish(str(self.pwm))
                time.sleep(0.01)

            for a in range(0,1000,1):
                self.A = "!M "+ str(a)
                self.ser.write(self.A)
                self.ser.write("\r")
                self.pwm = "pwm: " + str(a)
                self.pub.publish(str(self.pwm))
                time.sleep(0.01)

if __name__ == '__main__':
    #rospy.init_node("MOTORES")
    rospy.init_node('motor',anonymous=True)
    cv = MOTORES()
