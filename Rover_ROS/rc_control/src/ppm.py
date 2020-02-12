#!/usr/bin/env python
import time
import pigpio # http://abyz.me.uk/rpi/pigpio/python.html

pi = pigpio.pi()

class RC(object):

	def __init__(self):
		self.start_of_frame = False
		self.channel = 0

		self.last_tick = None
		self.diff = 0

		self.last_tick_A = None
		self.diff_A = 0

		self.modo = 0

		pi.set_mode(24, pigpio.INPUT)
		pi.set_mode(23, pigpio.INPUT)
		pi.set_mode(18, pigpio.INPUT)

		pi.set_mode(12, pigpio.OUTPUT)
		pi.set_mode(13, pigpio.OUTPUT)

                self.M1 = pi.hardware_PWM(12, 1000, 0)
                self.M2 = pi.hardware_PWM(13, 1000, 0)
		#self.cbf1()

	def stop(self):
		x = 1
		print "pailas"

	def cbf1(self, gpio, level, tick):
		if self.last_tick is not None:
			self.diff = pigpio.tickDiff(self.last_tick, tick)
			if self.diff < 3000:
				self.modo = self.diff
				modo = self.diff

				if 1900 < modo < 2100:
					self.stop()

				#if 1400 < modo < 1600:
				#	self.cbf1

				if 900 < modo < 1000:
					self.avanzar()

		#print self.modo
		self.last_tick = tick

	def avanzar(self, gpio, level, tick):
		#print "Hola"
                if self.last_tick_A is not None:
                        self.diff_A = pigpio.tickDiff(self.last_tick_A, tick)
                        if self.diff_A < 3000:
                                print self.diff_A
				x = 1

		self.last_tick_A = tick
		#pi.callback(24, pigpio.EITHER_EDGE, self.cbf1)

if __name__== '__main__':

	while True:
		cv = RC()
		pi.callback(24, pigpio.EITHER_EDGE, cv.cbf1)
		pi.callback(23, pigpio.EITHER_EDGE, cv.avanzar)

		#RC.cbf1

		#pi.callback(23, pigpio.EITHER_EDGE, cv.avanzar)

		#cb = pi.callback(23, pigpio.EITHER_EDGE, cv.cbf())
		#cb1 = pi.callback(18, pigpio.EITHER_EDGE, read)
		time.sleep(1)
		#pi.cancel()
		#pi.stop()
