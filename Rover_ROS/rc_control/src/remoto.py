#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import numpy as np
import time
import pigpio
from sensor_msgs.msg import JointState

msg = JointState()
pi = pigpio.pi()

diff_T = 0
diff_S = 0
diff_A = 0

diffT = 0
diffS = 0
diffA = 0

last_tick_T = None
last_tick_S = None
last_tick_A = None

error_1 = 0

cuenta = 0

class RC(object):

        def __init__(self):
                self.pub = rospy.Publisher('/PWM', JointState, queue_size=10)

                # Variables
                self.st = 0
                self.ax = 0
                self.th = 0
                self.refT = 1535
                self.refS = 1535

		self.signal = [90 ,
90 ,
90 ,
90 ,
90 ,
90 ,
90 ,
90 ,
160 ,
90 ,
90 ,
160 ,
90 ,
90 ,
160 ,
160 ,
90 ,
90 ,
90 ,
90 ,
160 ,
160 ,
160 ,
90 ,
90 ,
160 ,
90 ,
160 ,
90 ,
90 ,
90 ,
160 ,
160 ,
90 ,
160 ,
160 ,
160 ,
160 ,
90 ,
90 ,
90 ,
90 ,
160 ,
90 ,
90 ,
90 ,
160 ,
90 ,
90 ,
90 ,
90 ,
160 ,
90 ,
160 ,
160 ,
160 ,
160 ,
160 ,
90 ,
160 ,
160 ,
160 ,
160 ,
160 ,
160 ,
90 ,
160 ,
90 ,
90 ,
160 ,
90 ,
90 ,
90 ,
90 ,
90 ,
160 ,
160 ,
90 ,
90 ,
160 ,
90 ,
90 ,
90 ,
160 ,
160 ,
160 ,
90 ,
160 ,
90 ,
90 ,
90 ,
160 ,
90 ,
160 ,
160 ,
90 ,
90 ,
160 ,
160 ,
90 ,
160 ,
90 ,
160 ,
90 ,
160 ,
90 ,
160 ,
160 ,
90 ,
160 ,
160 ,
90 ,
160 ,
160 ,
160 ,
90 ,
160 ,
90 ,
160 ,
160 ,
160 ,
160 ,
90 ,
160 ,
160 ,
90 ,
90 ,
160 ,
90 ,
160 ,
160 ,
160 ,
90 ,
90 ,
90 ,
90 ,
90 ,
160 ,
90 ,
160 ,
90 ,
90 ,
160 ,
90 ,
160 ,
160 ,
90 ,
160 ,
90 ,
160 ,
160 ,
90 ,
90 ,
90 ,
90 ,
90 ,
90 ,
160 ,
160 ,
160 ,
160 ,
160 ,
160 ,
160 ,
90 ,
90 ,
160 ,
90 ,
90 ,
160 ,
90 ,
160 ,
90 ,
160 ,
160 ,
160 ,
90 ,
160 ,
160 ,
90 ,
160 ,
90 ,
90 ,
90 ,
90 ,
160 ,
160 ,
90 ,
160 ,
90 ,
90 ,
160 ,
160 ,
160 ,
90 ,
160 ,
160 ,
160 ,
90 ,
90]

                self.valor = 0
                self.cuenta = 0

                # pins mode
                pi.set_mode(13, pigpio.OUTPUT)
                pi.set_mode(12, pigpio.OUTPUT)
                pi.set_mode(24, pigpio.INPUT)
                pi.set_mode(23, pigpio.INPUT)
                pi.set_mode(18, pigpio.INPUT)

                # hardware_PWM(gpio, PWMfreq, PWMduty)
                self.M1 = pi.hardware_PWM(12, 1000, 0)
                self.M2 = pi.hardware_PWM(13, 1000, 0)
                #self.rate = rospy.Rate(10)

        def stop(self, ax):
                x = 1
                pi.set_PWM_dutycycle(13, 125.5)
                pi.set_PWM_dutycycle(12, 125.5)

        def Throttle(self, th, st, ax):
                delta = th - self.refT
                if abs(delta) > 600:
                        th = self.refT

                delta = st - self.refS
                if abs(delta) > 600:
                        st = self.refS

                #Ecuaciones para linealizar y aplicar un PWM de 0 a 255
                RC1 = (2095-th)/1130.0 * 255
                RC2 = (1920-st)/795.0 * 255

                #Saturacion de a los motores
                if RC1 >= 255.0:
                        RC1 = 255.0

                if RC2 >= 255.0:
                        RC2 = 255.0

                if RC1 <= 0.0:
                        RC1 = 0.0

                if RC2 <= 0.0:
                        RC2 = 0.0

                #Aplicar PWM a los motores
                pi.set_PWM_dutycycle(13, RC1)
                pi.set_PWM_dutycycle(12, RC2)

                #Publicar mensaje
                #msg.velocity = [RC1,RC2]
                #msg.header.stamp = rospy.get_rostime()
                #self.pub.publish(msg)

                # update variables
                self.refT = th
                self.refS = st

                #print str(RC1) + "\t" + str(RC2)
                #print RC2

        def automatico(self, ax):
		global cuenta
                x = 1
                pi.set_PWM_dutycycle(13, 92.0)
                #pi.set_PWM_dutycycle(12, self.signal[cuenta])
                time.sleep(0.1)
                cuenta = cuenta + 1

		if cuenta == 200:
			pi.set_PWM_dutycycle(13, 125.5)
                	pi.set_PWM_dutycycle(12, 125.5)

                print cuenta
		#print ax

if __name__== '__main__':

        while True:
                try:
                        rospy.init_node('remoto',anonymous=True, disable_signals=True)
                        cv = RC()

                        def Throttle(gpio, level, tick):
                                global last_tick_T, diff_T, diffT, diffS, diffA
                                if last_tick_T is not None:
                                        diff_T = pigpio.tickDiff(last_tick_T, tick)

                                        if diff_T < 3000 and diff_S < 3000 and diff_A < 3000:
                                                diffT = diff_T
                                                diffS = diff_S
                                                diffA = diff_A

                                	if 1900 < diffA < 2100:
                                        	cv.stop(diffA)

                                	if 1400 < diffA < 1600:
                                        	cv.Throttle(diffT, diffS, diffA)

                                	if 900 < diffA < 1000:
                                        	cv.automatico(diffA)

                                #if 900 < diffA < 1000:
                                #               cv.AX1(diffA)
                                # update the tick
                                last_tick_T = tick

                        def Steering(gpio, level, tick):
                                global last_tick_S, diff_S
                                if last_tick_S is not None:
                                        diff_S = pigpio.tickDiff(last_tick_S, tick)

                                last_tick_S = tick
                                #cv.Throttle(diff_S)

                        def AX1(gpio, level, tick):
                                global last_tick_A, diff_A, diffA
                                if last_tick_A is not None:
                                        diff_A = pigpio.tickDiff(last_tick_A, tick)

                                last_tick_A = tick
                                #cv.AX1(diff_A)

                        # Funciones que leen la senal a travez de callback
                        cb = pi.callback(23, pigpio.EITHER_EDGE, Throttle)
                        cb1 = pi.callback(18, pigpio.EITHER_EDGE, Steering)
                        cb2 = pi.callback(24, pigpio.EITHER_EDGE, AX1)

                        time.sleep(1)
                        cb.cancel()
                        cb1.cancel()
                        cb2.cancel()

                except rospy.ROSInterruptException:
                        #GPIO.cleanup()
                        print "closed"
                        pass

