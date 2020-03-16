#! /usr/bin/env python
import serial
import rospy
import pigpio
import RPi.GPIO as GPIO

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

		self.avanzar = 72.0

                # pins mode
                pi.set_mode(13, pigpio.OUTPUT)
                pi.set_mode(12, pigpio.OUTPUT)

                # hardware_PWM(gpio, PWMfreq, PWMduty)
                self.M1 = pi.hardware_PWM(12, 1000, 0)
                self.M2 = pi.hardware_PWM(13, 1000, 0)
		self.signal = [150,
150,
100,
100,
150,
150,
100,
100,
150,
100,
100,
150,
100,
100,
150,
150,
150,
100,
100,
100,
150,
150,
150,
100,
100,
150,
100,
150,
150,
100,
100,
150,
150,
100,
150,
150,
150,
150,
100,
100,
100,
100,
150,
100,
100,
100,
150,
100,
100,
100,
100,
150,
100,
150,
150,
150,
150,
150,
100,
150,
150,
150,
150,
150,
150,
100,
150,
100,
100,
150,
100,
100,
100,
100,
100,
150,
150,
100,
100,
150,
100,
100,
100,
150,
150,
150,
100,
150,
100,
100,
100,
150,
100,
150,
150,
100,
100,
150,
150,
100,
150,
100,
150,
100,
150,
100,
150,
150,
100,
150,
150,
100,
150,
150,
150,
100,
150,
100,
150,
150,
150,
150,
100,
150,
150,
100,
100,
150,
100,
150]

		self.signal2 = [160,
160,
160,
160,
90,
90,
90,
90,
160,
160,
160,
160,
90,
90,
90,
90,
160,
160,
160,
160,
90,
90,
90,
90,
160,
160,
160,
160,
90,
90,
90,
90,
160,
160,
160,
160,
90,
90,
90,
90,
160,
160,
160,
90,
90,
160,
160,
90,
160,
160,
90,
90,
160,
160,
160,
160,
90,
90,
90,
90,
160,
160,
160,
160,
90,
90,
90,
90,
160,
160,
160,
90,
90,
90,
160,
160,
160,
90,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160,
90,
90,
160,
160]



		self.signal1 = [96.9986973747417,
160.367263157586,
103.044198885187,
127.440201859742,
168.610498360666,
162.397192034520,
101.026298169273,
147.145040225513,
160.0310596289714,
71.7987481915531,
144.758818512463,
71.0659845137895,
58.4767174802566,
122.528639129170,
167.0912806160113,
161.9167981601634,
78.2968937965976,
125.528708906714,
149.126763780877,
154.397040649398,
90.486714063968,
95.273195494008,
150.773565234267,
80.4720369986287,
125.603278143221,
81.8458811157649,
162.387012044691,
178.076315997994,
62.6865353802674,
150.132413308573,
113.812672364319,
109.647075232337,
96.9743440399086,
163.322756923847,
136.191096393096,
120.715613321905,
137.240511502348,
155.610505068563,
97.5779880485117,
99.8847228540834,
145.601529488855,
173.255422187249,
162.914337978202,
120.816216507171,
83.3297801251866,
52.1661692877867,
95.8196384356972,
159.196680804478,
107.453511369024,
133.125104605634,
130.938107453090,
56.4827573871981,
162.941899042882,
186.787905049806,
123.180584327590,
155.159305722366,
101.035879423237,
142.754764002659,
189.824385014830,
164.811413630458,
102.804904380901,
92.9515181529459,
126.016696100607,
117.073767252768,
122.445775908742,
163.559480308899,
131.195762286723,
175.670448480737,
123.055343724734,
120.272739364209,
148.828668973987,
137.518704821049,
117.909616257697,
75.4269764506960,
89.9688137954152,
143.393114533630,
141.130957336311,
79.1338999699948,
128.075503490504,
140.696076806375,
88.1544332511151,
93.6110794472734,
161.184015352868,
70.3245126411921,
138.143089523367,
154.526695143915,
69.1667893177398,
120.366641186265,
88.1546482624339,
127.152456786921,
111.366906724911,
73.4920497262757,
143.427439885843,
142.755230614454,
105.317144219758,
140.650454554788,
82.9429951849758,
86.3197903891167,
88.8265206140667,
121.009647940982,
108.222025324514,
183.292323476313,
122.187782130191,
141.697218407978,
142.074384580398,
135.837625189261,
130.288181439919,
162.726509979714,
88.4960962285463,
174.784165562055,
145.534562409436,
88.9483739844983,
158.598391041721,
160.830208045841,
177.005253870035,
111.423944793361,
174.910789477636,
132.500523181023,
79.2699209299884,
100.455249286415,
153.029540580179,
179.644654885845,
87.9106984021635,
120.952691841912,
104.135728479049,
182.319402725841,
96.7592128904307,
165.421112550958,
105.128017781129,
148.319436348718,
83.5614501983301,
170.623214592673,
56.4784534707154,
134.320530381654,
137.356323925631,
99.0826763738451,
94.5899240745394,
143.161727795466,
92.4399067693913,
123.509137537977,
37.4645800312190,
60.2343926169979,
123.000069481285,
158.838742853897,
73.5361271208358,
145.298378855232,
139.616262135336,
106.240884636185,
123.642074692211,
191.923337843862]

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
		print "stop"

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
                #print RC2

	def automatico(self):

		X = 1
		#print "automata"

		if 1900 < self.calibration < 2200:
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


		if 900 < self.calibration < 1000:
			if self.cuenta <130:
				pi.set_PWM_dutycycle(13, self.avanzar)
				pi.set_PWM_dutycycle(12, self.signal1[self.cuenta])
				msg.velocity = [self.avanzar,self.signal1[self.cuenta]]
				self.pub.publish(msg)
				print self.cuenta

			if self.cuenta == 130:
				pi.set_PWM_dutycycle(13, 125.5)
				pi.set_PWM_dutycycle(12, 125.5)
				print "Termino"

			self.cuenta = self.cuenta + 1

	def main(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
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

		#pi.set_PWM_dutycycle(13, 125.5)
		#pi.set_PWM_dutycycle(12, 125.5)
		#pass

