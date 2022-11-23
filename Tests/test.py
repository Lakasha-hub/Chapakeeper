from re import I
import pyfirmata
from pyfirmata import Arduino, SERVO, util
import time

arduino=Arduino("COM9")
arduino.digital[9].mode=SERVO
while True:
    v=int(input("Vallue: S"))
    arduino.digital[9].write(v)
    time.sleep(3)