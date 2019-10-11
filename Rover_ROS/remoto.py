#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
import serial
from std_msgs.msg import Float32

class RC(object):
    def __init__(self):
        # se define el puerto por el cual se va a realizar la comunicacion
        #self.ser = serial.Serial(
        #    port='/dev/ttyACM0',
        #    baudrate=115200,
        #    parity=serial.PARITY_ODD,
        #    stopbits=serial.STOPBITS_ONE,
        #    bytesize=serial.EIGHTBITS
        #)
        self.pub = rospy.Publisher('/PWM', Float32, queue_size=10)
        self.A = 0
        self.g = 0
        self.comienzo = 0.0
        self.modo = 0.0
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.IN)
        GPIO.setup(18, GPIO.IN)
        GPIO.setup(24, GPIO.IN)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(12, GPIO.OUT)

        self.M1 = GPIO.PWM(13, 100) # Declaracion de PINS de PWM a una frecuencia de 1kHZ
        self.M2 = GPIO.PWM(12, 100)
        self.M1.start(0) # Inicializa el Pin
        self.M2.start(0)
        #rospy.spin()

    def stop(self):
        while True:
                GPIO.wait_for_edge(24, GPIO.RISING)
                E = time.time()
                if GPIO.wait_for_edge(24, GPIO.FALLING):
                        F = time.time()
                modo = F-E

                if 0.0016 > modo > 0.0014:
                        self.rc()

                if 0.0010 > modo > 0.0008:
                        self.control()

                #print modo

    def rc(self):
        while True:

                GPIO.wait_for_edge(24, GPIO.RISING)
                C = time.time()
                if GPIO.wait_for_edge(24, GPIO.FALLING):
                        D = time.time()

                modo = D-C

                if 0.0021 > modo > 0.0019:
                         self.stop()

                if 0.0010 > modo > 0.0008:
                         self.control()

                #print modo

                # Avanzar (Vmax)
                GPIO.wait_for_edge(23, GPIO.RISING) # funcion que detecta flanco de subida
                start = time.time()         # funcion para guardar el tiempo en ese instante
                if GPIO.wait_for_edge(23, GPIO.FALLING): #funcion que detecta flanco de bajada
                        end = time.time()       # funcion para guardar el tiempo

                # Girar (W)
                GPIO.wait_for_edge(18, GPIO.RISING)
                inicio = time.time()
                if GPIO.wait_for_edge(18, GPIO.FALLING):
                        fin = time.time()

                duration = end - start      #seconds to run for loop
                tiempo = fin - inicio

                A = duration    #channel Throttle

                #print "A: %s" % A
                #print "\t"
                #print "B: %s" % B

                #Ecuaciones para linealizar y aplicar un PWM de 0 a 100
                # A cada motor
                A = (0.0021-A)/0.00111 * 100
                B = (B - 0.00112)/0.0008 * 100

                #A = (A-0.00099)/0.0011 *100 #200.0 - 100.0
                #B = (0.00192-B)/0.0008 *100 #800.0 - 400.0

                #WR = A*0.4 - (B*0.4*0.6)/2 + 50
                #WL = A*0.4 - (B*0.4*0.6)/2 + 50

                WR=B
                WL=A

                #print "WL: %s" % WL
                #print "\t"
                #print "WR: %s" % WR

                # saturacion para que los motores no reciban valores >100 y <0.0
                if WR > 99.9:
                        WR = 99.9

                if WR < 0.1:
                        WR = 0.1

                if WL > 99.9:
                        WL = 99.9

                if WL < 0.1:
                        WL = 0.1

                self.M1.ChangeDutyCycle(WL)     # Motor Izquierdo
                self.M2.ChangeDutyCycle(WR)     # Motor Derecho

                self.pub.publish(WL)
                self.pub.publish(WR)

                #print "WL: %s" % WL
                #print "\t"
                #print "WR: %s" % WR


    def control(self):
        while True:
                GPIO.wait_for_edge(24, GPIO.RISING)
                A = time.time()
                if GPIO.wait_for_edge(24, GPIO.FALLING):
                        B = time.time()

                modo = B-A

                if 0.0021 > modo > 0.0019:

                if 0.0016 > modo > 0.0014:
                         self.rc()

                #print modo

if __name__== '__main__':
    try:
        rospy.init_node('remoto',anonymous=True, disable_signals=True)
        cv = RC()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(24, GPIO.IN)
        GPIO.wait_for_edge(24, GPIO.RISING)
        A = time.time()
        if GPIO.wait_for_edge(24, GPIO.FALLING):
                B = time.time()
        modo = B - A

        if 0.0021 > modo > 0.0019:
                cv.stop()

        if 0.0016 > modo > 0.0014:
                cv.rc()

        if 0.0010 > modo > 0.0008:
                cv.control()

    except KeyboardInterrupt:
                    
