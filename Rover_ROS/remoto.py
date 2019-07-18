import rospy
import RPi.GPIO as GPIO
import time
import serial
from std_msgs.msg import Float32

class RC(object):
    def __init__(self):
        # se define el puerto por el cual se va a realizar la comunicacion
        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.pub = rospy.Publisher('/PWM', Float32, queue_size=10)
        self.A = 0
        self.g = 0
        #self.NUM_CYCLES = 1.0
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.IN)
        GPIO.setup(18, GPIO.IN)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(16, GPIO.OUT)

        self.M1 = GPIO.PWM(13, 1000)
        self.M2 = GPIO.PWM(16, 1000)
        self.M1.start(0)
        self.M2.start(0)

        self.control()

    def control(self):
            while True:
                    GPIO.wait_for_edge(23, GPIO.RISING) # funcion que detecta f$
                    start = time.time()         # funcion para guardar en ese i$
                    if GPIO.wait_for_edge(23, GPIO.FALLING): #funcion que detec$
                        end = time.time()       # funcion para guardar el tiempo

                    GPIO.wait_for_edge(18, GPIO.RISING)
                    inicio = time.time()
                    if GPIO.wait_for_edge(18, GPIO.FALLING):
                        fin = time.time()

                    duration = end - start      #seconds to run for loop
                    tiempo = fin - inicio

                    A = tiempo
                    B = duration

                    # Ecuaciones para linealizar y aplicar un pwm de 0 a +/- 10$
                    A = (0.00191-A)/0.00081 * 100.0
                    B = (0.00206-B)/0.00112 * 100.0

                    #print A
                    #print "\r"
                    #print B

                    MR = B+A-50
                    ML = B-A+50

                    #print MR
                    #print "\r"
                    #print ML

                    # saturacion para que los motores no tengan senales >1000 y$
                    if MR > 100.0:
                        MR = 100.0

                    if MR < 0.0:
                        MR = 0.0

                    if ML > 100.0:
                        ML = 100.0

                    if ML < 0.0:
                        ML = 0.0

                    #self.A = "!M " + str(-ML) + str(" ") + str(-MR)
                    #self.ser.write(self.A)
                    #self.ser.write("\r")

                    self.M1.ChangeDutyCycle(MR)
                    self.M2.ChangeDutyCycle(ML)

                    print MR
                    print "\r"
                    print ML
                    #print self.A

if __name__== '__main__':
    try:
        rospy.init_node('remoto',anonymous=True, disable_signals=True)
        cv = RC()
    except KeyboardInterrupt:
        print "closed"
