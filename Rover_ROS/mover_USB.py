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
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.IN)
        GPIO.setup(18, GPIO.IN)
        self.control()

    def control(self):
            while True:
                    GPIO.wait_for_edge(23, GPIO.RISING) # funcion que detecta flaco de subida
                    start = time.time()         # funcion para guardar en ese instante de tiempo
                    if GPIO.wait_for_edge(23, GPIO.FALLING): #funcion que detecta flaco de bajada
                        end = time.time()       # funcion para guardar el tiempo

                    GPIO.wait_for_edge(18, GPIO.RISING)
                    inicio = time.time()
                    if GPIO.wait_for_edge(18, GPIO.FALLING):
                        fin = time.time()

                    duration = end - start      #seconds to run for loop

                    tiempo = fin - inicio
                    A = tiempo
                    B = duration

                    # Ecuaciones para linealizar y aplicar un pwm de 0 a +/- 1000
                    A = (A-0.0011)/0.00081 * 2000.0 -1000.0
                    B = (B-0.00094)/0.00112 * 2000.0 -1000.0

                    MR = B+A
                    ML = B-A

                    # saturacion para que los motores no tengan senales >1000 y <1000
                    if MR > 1000.0:
                        MR = 1000.0

                    if MR < -1000.0:
                        MR = -1000.0

                    if ML > 1000.0:
                        ML = 1000.0

                    if ML < -1000.0:
                        ML = -1000.0

                    self.A = "!M " + str(-ML) + str(" ") + str(-MR)
                    self.ser.write(self.A)
                    self.ser.write("\r")

                    print self.A

if __name__== '__main__':
    try:
        rospy.init_node('remoto',anonymous=True, disable_signals=True)
        cv = RC()
    except KeyboardInterrupt:
        print "closed"
