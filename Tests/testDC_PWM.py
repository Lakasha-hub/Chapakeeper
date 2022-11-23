import cv2
from pyfirmata import Arduino, util
import pyfirmata
import time
import asyncio
import numpy as np

#Función Principal (Main)
def main():
    #Llamo placa arduino mediante su puerto
    arduino=Arduino("COM5")
    #Bridge1 (Pata 1 de Puente H) = Pin 3 de Arduino
    bridge1 = arduino.digital[3]
    #Bridge2 (Pata 2 de Puente H)= Pin 5 de Arduino
    bridge2 = arduino.digital[5]
    #PWM del Motor = Pin 10 de Arduino
    motor = arduino.get_pin('d:10:p')
    #Pote = Pin analogico 0 de Arduino
    pote = arduino.get_pin('a:0:i')

    #Iterador de Arduino
    it = pyfirmata.util.Iterator(arduino)
    it.start()
    
    #Indico la dirección de giro
    bridge1.write(1)
    bridge2.write(0)

    #Ciclo del motor (Infinito)
    while True:
        #Lectura = analogRead(pote) o lectura del potenciometro
        lectura = pote.read()
        #Verifica que lectura sea distinto de None
        if lectura == None:
            continue
        #Imprimo valor de lectura
        print(lectura)
        #La velocidad del motor será el valor de lectura (0 a 255) o en Python (0 a 1)
        motor.write(lectura)

#Ejecuto la función principal:
if __name__=="__main__":
    main()