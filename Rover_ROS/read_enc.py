#! /usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class roboteq(object):
        def __init__(self):
                # se define el puerto de lectura
                self.driver = serial.Serial(
                                port = "/dev/ttyACM0",
                                baudrate=115200,
                                parity=serial.PARITY_ODD,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS
                                )

                self.pub = rospy.Publisher('/enc_data', Odometry, queue_size=10)

                #Batery voltage
                self.voltage_battery = 0.0

                #Temperature motors
                self.T_left = 0
                self.T_right = 0

                #Current motors
                self.I_left = 0.0
                self.I_right = 0.0

                #Encoders data
                self.enc_left = 0
                self.enc_right = 0
                self.counter = 0
                self.Ts_roboteq = 0.0

                self.read_roboteq()
                #rospy.spin()
                #self.rate = rospy.Rate(10)

        def read_roboteq(self):
                while True:
                        if self.driver.isOpen():
                                self.driver.write("EELD\r")
                                odome = self.driver.readline()
                                data = odome.split(",")
                                odom = Odometry()
                                #print odome

                                if len(data) == 8:
                                        #Battery Voltage
                                        battery = data[0]
                                        b_battery = battery.split("A")
                                        self.voltage_battery = b_battery[1]

                                        #Motor temperature
                                        self.T_left = int(data[2])
                                        self.T_right = int(data[1])

                                        #Motor current
                                        self.I_left = int(data[4])
                                        self.I_right = int(data[3])

                                        #pulses encoder
                                        self.enc_left = int(data[6])
                                        self.enc_right = float(data[5])

                                        #Counter
                                        counter = data[7]
                                        c_counter = counter.split("A")
                                        self.counter = c_counter[0]

                                        #ubicate in odom matrix
                                        odom.pose.covariance[0] = float(self.voltage_battery) # Voltaje de la bateria
                                        odom.pose.covariance[1] = int(self.T_left) # Temperatura motor izquierdo
                                        odom.pose.covariance[2] = int(self.T_right) # Temperatura motor derecho
                                        odom.pose.covariance[3] = float(self.I_left) # Corriente motor izquierdo
                                        odom.pose.covariance[4] = float(self.I_right) # Corriente motor izquierdo
                                        odom.pose.covariance[5] = int(self.enc_left) # pulsos enc izquierdo
                                        odom.pose.covariance[6] = int(self.enc_right) # pulsos enc derecho
                                        odom.pose.covariance[7] = int(self.counter) # contador del programa

                                odom.header.stamp = rospy.get_rostime()
                                self.pub.publish(odom)
                                #self.rate.sleep()
                                #print self.enc_left

if __name__=='__main__':
        try:
                rospy.init_node('Odometria',anonymous=True, disable_signals=True)
                print "Nodo creado"
                cv = roboteq()
        except rospy.ROSInterruptException:
                pass
