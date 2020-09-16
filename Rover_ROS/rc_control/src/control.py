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
diff_A1 = 0
diff_A2 = 0

diffT = 0
diffS = 0
diffA1 = 0
diffA2 = 0

last_tick_T = None
last_tick_S = None
last_tick_A1 = None
last_tick_A2 = None

class RC(object):

	def __init__(self):
       		self.pub = rospy.Publisher('/channels', JointState, queue_size=10)
		self.ax1 = 0
		self.ax2 = 0
		self.st = 0
		self.th = 0

                pi.set_mode(24, pigpio.INPUT)
                pi.set_mode(23, pigpio.INPUT)
                pi.set_mode(18, pigpio.INPUT)
		pi.set_mode(4, pigpio.INPUT)

		self.cont = 0

	def ch(self, ax1, ax2, st, th):
		self.ax1 = ax1
		self.ax2 = ax2
		self.st = st
		self.th = th

		#rate = rospy.Rate(10)

		#while not rospy.is_shutdown():
		#print self.ax1
		msg.velocity = [self.ax1, self.ax2, self.st, self.th]
		msg.header.stamp = rospy.get_rostime()
		self.pub.publish(msg)

		#rate.sleep()

if __name__== '__main__':

	while True:
		try:
			rospy.init_node('remoto',anonymous=True, disable_signals=True)
			cv = RC()

                        def AX1(gpio, level, tick):
                                global last_tick_A1, diff_A1, diffA1, diff_A2, diffA2 ,diffS, diffT, diff_S, diff_T
                                if last_tick_A1 is not None:
                                        diff_A1 = pigpio.tickDiff(last_tick_A1, tick)
                                        if diff_A1 < 3000 and diff_S < 3000 and diff_T < 3000 and diff_A2:
                                                diffA1 = diff_A1
						diffA2 = diff_A2
                                                diffS = diff_S
                                                diffT = diff_T
                                		cv.ch(diffA1, diffA2, diffS, diffT)

                                last_tick_A1 = tick


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

			def AX2(gpio, level, tick):
				global last_tick_A2, diff_A2
				if last_tick_A2 is not None:
					diff_A2 = pigpio.tickDiff(last_tick_A2, tick)

				last_tick_A2 = tick


			cb = pi.callback(24, pigpio.EITHER_EDGE, AX1)
			cb1 = pi.callback(23, pigpio.EITHER_EDGE, Throttle)
			cb2 = pi.callback(18, pigpio.EITHER_EDGE, Steering)
			cb3 = pi.callback(4, pigpio.EITHER_EDGE, AX2)

			time.sleep(5)
			cb.cancel()
			cb1.cancel()
			cb2.cancel()
			cb3.cancel()

		except rospy.ROSInterruptException:
        		GPIO.cleanup()
        		print "closed"
			pass
