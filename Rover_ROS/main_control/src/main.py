#! /usr/bin/env python
import serial
import rospy
import pigpio
import scipy.io
import sys

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

pi = pigpio.pi()
msg = JointState()

class adq_datos(object):

        def __init__(self):
		self.pub = rospy.Publisher('/main_node', JointState, queue_size=10)

		self.modo = 0
		self.calibration = 0
		self.th = 0
		self.st = 0
		self.refT = 1535
		self.refS = 1535

		self.RC1 = 0
		self.RC2 = 0
		self.cuenta = 0
		self.calibra = 0

		self.avanzar = 60.0

                # pins mode
                pi.set_mode(13, pigpio.OUTPUT)
                pi.set_mode(12, pigpio.OUTPUT)

                # hardware_PWM(gpio, PWMfreq, PWMduty)
                self.M1 = pi.hardware_PWM(12, 1000, 0)
                self.M2 = pi.hardware_PWM(13, 1000, 0)

		#VARIABLES PARA LA SENAL PRBS
		self.signal = scipy.io.loadmat('matlab.mat')
		self.data = self.signal['u']
		self.cuenta = 0
		self.ini = True

                rospy.Subscriber('/channels', JointState, self.synchronize_pwm)

        def synchronize_pwm(self, pwm):
		self.modo = pwm.velocity[0]		# AX1
		self.calibration = pwm.velocity[1]	# AX2
               	self.st = pwm.velocity[2]		# Stering
               	self.th = pwm.velocity[3]		# Throttle

	def stop(self):
		x = 1
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

		print "STOP"

	def manual(self):
		x = 1
		#print "manual"

                delta = self.th - self.refT
                if abs(delta) > 600:
                        self.th = self.refT

                delta = self.st - self.refS
                if abs(delta) > 600:
                        self.st = self.refS

                #Ecuaciones para linealizar y aplicar un PWM de 0 a 255
                self.RC1 = (2095-self.th)/1130.0 * 255
                self.RC2 = (1920-self.st)/795.0 * 255

                #Saturacion de a los motores
                if self.RC1 >= 254.9:
                        self.RC1 = 254.9

                if self.RC2 >= 254.9:
                        self.RC2 = 254.9

                if self.RC1 <= 0.1:
                        self.RC1 = 0.1

                if self.RC2 <= 0.1:
                        self.RC2 = 0.1

                #Aplicar PWM a los motores
                pi.set_PWM_dutycycle(13, self.RC1)
                pi.set_PWM_dutycycle(12, self.RC2)

                #Publicar mensaje
                msg.velocity = [self.RC1,self.RC2]
		#msg.name = ['Hola']
                msg.header.stamp = rospy.get_rostime()
                self.pub.publish(msg)

                # update variables
                self.refT = self.th
                self.refS = self.st

		print str(self.RC1) + "\t" + str(self.RC2)

	def automatico(self):

		if 900 < self.calibration < 1000:
			if self.calibra < 54:
				pi.set_PWM_dutycycle(12,180)
				print "calibrando"

			self.calibra = self.calibra + 1

			if self.calibra == 54:
				pi.set_PWM_dutycycle(12, 125.5)
				pi.set_PWM_dutycycle(13, 125.5)
				msg.name = ['Termino']
				self.pub.publish(msg)
				msg.name = []


		if 1900 < self.calibration < 2000:
			if self.cuenta + 1 > len(self.data):
				pi.set_PWM_dutycycle(13, 127.5)
				pi.set_PWM_dutycycle(12, 127.5)
				#sys.exit(0)

			else:
				pi.set_PWM_dutycycle(13, self.avanzar)
				pi.set_PWM_dutycycle(12, self.data[self.cuenta])

			self.cuenta += 1

			if self.cuenta == 30:
				self.cuenta = 60
				self.ini = False

			if self.cuenta == 90:
				self.cuenta = 60


			print self.cuenta


	def main(self):
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			pi.set_PWM_dutycycle(13, self.avanzar)
			pi.set_PWM_dutycycle(12, 127.5)

			if 1900 < self.modo < 2100:
				self.stop()

                        if 1400 < self.modo < 1600:
                                self.manual()

                        if 900 < self.modo < 1000:
                                self.automatico()

			#print self.modo
			rate.sleep()

if __name__=='__main__':
       	try:
		rospy.init_node('main',anonymous=True, disable_signals=True)
		cv = adq_datos()
		print "Nodo PWM creado"
		cv.main()

	except AttributeError:
		print "error_de_atributo"
		pi.set_PWM_dutycycle(13, 125.5)
		pi.set_PWM_dutycycle(12, 125.5)

	except pigpio.error:
                print "error_pigpio"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except IndexError:
                print "error_IndexdeLista"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except rospy.ROSInterruptException:
		print "ROSInterrupt"
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

	except KeyboardInterrupt:
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)
		print "keyboardInterrupt"

	except SyntaxError:
		pi.set_PWM_dutycycle(13, 125.5)
		pi.set_PWM_dutycycle(12, 125.5)
		print "SyntaxError"

