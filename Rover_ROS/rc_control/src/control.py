#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
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

class RC(object):

	def __init__(self):
       		self.pub = rospy.Publisher('/channels', JointState, queue_size=10)
		self.ax = 0
		self.st = 0
		self.th = 0

                pi.set_mode(24, pigpio.INPUT)
                pi.set_mode(23, pigpio.INPUT)
                pi.set_mode(18, pigpio.INPUT)

	def ch(self, ax, st, th):
		self.ax = ax
		self.st = st
		self.th = th

		msg.velocity = [self.ax, self.st, self.th]
		msg.header.stamp = rospy.get_rostime()
		self.pub.publish(msg)

if __name__== '__main__':

	while True:
		try:
			rospy.init_node('remoto',anonymous=True, disable_signals=True)
			cv = RC()

                        def AX1(gpio, level, tick):
                                global last_tick_A, diff_A, diffA, diffS, diffT, diff_S, diff_T
                                if last_tick_A is not None:
                                        diff_A = pigpio.tickDiff(last_tick_A, tick)
                                        if diff_A < 3000 and diff_S < 3000 and diff_T < 3000:
                                                diffA = diff_A
                                                diffS = diff_S
                                                diffT = diff_T
                                		cv.ch(diffA, diffS, diffT)

                                last_tick_A = tick


                        def Throttle(gpio, level, tick):
                                global last_tick_T, diff_T
                                if last_tick_T is not None:
                                        diff_T = pigpio.tickDiff(last_tick_T, tick)

				last_tick_T = tick


                        def Steering(gpio, level, tick):
                                global last_tick_S, diff_S
                                if last_tick_S is not None:
                                        diff_S = pigpio.tickDiff(last_tick_S, tick)

                                last_tick_S = tick


			cb = pi.callback(24, pigpio.EITHER_EDGE, AX1)
			cb1 = pi.callback(23, pigpio.EITHER_EDGE, Throttle)
			cb2 = pi.callback(18, pigpio.EITHER_EDGE, Steering)

			time.sleep(5)
			cb.cancel()
			cb1.cancel()
			cb2.cancel()

		except rospy.ROSInterruptException:
        		GPIO.cleanup()
        		print "closed"
			pass
