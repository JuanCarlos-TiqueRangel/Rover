
import time
import serial


ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
if ser.isOpen():
    #ser.close()
    cuenta = 0
    for i in range(1000):
        A = "!M -"+ str(cuenta)
        ser.write(A)
        ser.write("\r")
        cuenta = i + 1
        time.sleep(0.01)

        #ser.write("?A \r")
        # out=''
        # time.sleep(0.1)
        # while ser.inWaiting() > 0:
        #     out += ser.read(50)
        #
        # if out != '':
        #     print(">>" + out)
        #


    for i in range(1000):
        A = "!M "+ str(cuenta)
        ser.write(A)
        ser.write("\r")
        cuenta = cuenta-1
        time.sleep(0.01)
