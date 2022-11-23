#Importo OpenCV
import cv2
#Importo PyFirmata y tipo de placa Arduino Uno
from pyfirmata import Arduino, util
import pyfirmata
import time
import asyncio
import numpy as np

#Función map que me permite acomodar valores modificando su escala
def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#Función Asíncrona que mueve el motor a una posición determinada por la cámara
async def moveMotor(prev_position, NewValue, bridge1, bridge2, pote):

    #prev_position (Ultima posición que obtuvo el Motor en grados)
    #NewValue (Valor que obtengo de la cámara en grados (Posición del Objeto))
    #bridge1 y bridge2 (Pines del Puente H (Permiten el sentido de giro del Motor))
    #pote (Pin analogico que determina la prev_position)

    #Si la posición es distinta de None y el valor que tiene es mayor o igual a NewValue:
    if prev_position != None and prev_position >= NewValue:
        #Gira hacia un sentido denominado por nosotros como (Dirección A)
        bridge1.write(0)
        bridge2.write(1)
        #Imprimo la dirección que se realizó
        print('Direccion A')
        #Obtengo la posición del motor
        prev_position = pote.read()
        #Si la posición del motor es igual a NewValue o tiene un margen de error de + o - 10:
        if prev_position == NewValue or NewValue - 10 < prev_position < NewValue + 10:
            #Detengo el Motor
            bridge1.write(0)
            bridge2.write(0)
    #Si la posición es distinta de None y el valor que tiene es menor o igual a NewValue:
    elif prev_position != None and prev_position <= NewValue:
        #Gira hacia un sentido denominado por nosotros como (Dirección B)
        bridge1.write(1)
        bridge2.write(0)
        #Imprimo la dirección que se realizó
        print('direccion B')
        #Obtengo la posición del motor
        prev_position = pote.read()
        #Si la posición del motor es igual a NewValue o tiene un margen de error de + o - 10:
        if prev_position == NewValue or NewValue - 10 < prev_position < NewValue + 10:
            #Detengo el Motor
            bridge1.write(0)
            bridge2.write(0)

#Función Principal o Main de tipo Asíncronica
async def main():
    #Establece conexion con arduino a traves del puerto
    arduino=Arduino("COM9")
    #Selecciono los puertos que van hacia el puente H(L293d) del Arduino
    bridge1 = arduino.digital[3]
    bridge2 = arduino.digital[5]
    #Selecciono el pin que define el PWM al motor
    motor = arduino.get_pin('d:10:p')
    #Selecciono el pin donde se tomaran los valores del potenciometro
    pote = arduino.get_pin('a:0:i')

    #Iterador de Arduino
    it = pyfirmata.util.Iterator(arduino)
    it.start()

    #Indico que la captura de imagenes será con la WebCam 
    webcam_video=cv2.VideoCapture(1)
    #Defino la Escala de colores por los cuales se basará la detección
    lowerColor = np.array([ 0, 155, 165])
    upperColor = np.array([30, 255, 255])
    #PWM del Motor en su maxima velocidad 1 == 255
    speed = 1    
    motor.write(speed)

    while True:
        #Verifico que la Webcam este conectada
        success, video, = webcam_video.read()
        #Si la conexión es exitosa
        if success==True:
            #Obtengo la posición del motor
            prev_position = pote.read()
            #Convierto su valor mediante la función Map de 0° a 165°
            prev_position = _map(prev_position, 0, 1, 0, 165)
            print(prev_position)
            #Recorto el área de detección a 600 x 600px
            video=cv2.resize(video,(600,600))
            #Aplico un filtro de desenfoque para evitar fallas de detección
            img=cv2.GaussianBlur(video,(11,11),10)
            #Aplico un flip al la lectura de la WebCam para evitar el espejo
            video=cv2.flip(video, 1)
            #Ajusto los valores de la lectura para obtener un mejor contraste en imágenes con poco contraste debido al deslumbramiento.
            video = cv2.normalize(video, None, alpha=0,beta=350, norm_type=cv2.NORM_MINMAX)
            #Convierto los colores de la lectura en una escala de grises
            img=cv2.cvtColor(video, cv2.COLOR_BGR2HSV)
            #mask es el objeto a detectar el cual defino los parametros de deteccion mediante los colores
            mask=cv2.inRange(img, lowerColor, upperColor)
            #Busco los contornos del objeto 
            mask_contours, hierarchy=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            #Si la lectura es distinta de 0, es decir detecta un objeto:
            if len(mask_contours)!=0:
                #Por cada objeto que detecta:
                for mask_contour in mask_contours:
                    #Si el objeto tiene un área mayor a 500 y menor a 800:
                    if cv2.contourArea(mask_contour) > 500 <800:
                        #Creo un Rectangulo que rodea el objeto
                        x, y, w, h = cv2.boundingRect(mask_contour)
                        cv2.rectangle(video, (x, y), (x + w, y + h), (0, 0, 255), 3)
                        cnt=mask_contours
                        M = cv2.moments(mask_contour)
                        if (M["m00"]==0): M["m00"]=1
                        x = int(M["m10"]/M["m00"])
                        y = int(M['m01']/M['m00'])
                        cv2.circle(video, (x, y), 7, (255, 255, 255), -1)
                        #Muestro la ubicación del objeto en el eje X e Y
                        cv2.putText(video, f"x:{x} y:{y}", (x - 20, y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (39 , 236, 230), 2)


                        errorX=x-300

                        OldMax=300
                        OldMin=-300
                        NewMax=165
                        NewMin=0

                        NewValue = _map(errorX, OldMin, OldMax, NewMin, NewMax)
                        
                        if NewValue>165:
                            NewValue=165

                        if errorX>0:
                            if y>400:
                                NewValue+=10
                        if errorX<0:
                            if y>400:
                                NewValue-=10
                                
                        if NewValue<0:
                            NewValue=0
                        #print(int(NewValue))

                        #Si la posición del Motor es distinta de None 
                        if prev_position == None:
                            continue
                        #Ejecuto la función Asíncronica que mueve el motor
                        await asyncio.create_task(moveMotor(prev_position, NewValue, bridge1, bridge2, pote))
                        # print(prev_position)
                        #Imprimo el contorno del objeto que se esta detectando:
                        # print(cv2.contourArea(mask_contour))
                        #Si el area de contorno es mayor a 25000:
                        if cv2.contourArea(mask_contour) > 25000:
                            #Detengo el motor ya que la pelota se encuentra cerca y no queremos que el motor oscile
                            bridge1.write(0)
                            bridge2.write(0)
                            #Esta parada dura 3 segundos
                            await asyncio.sleep(3)
                    #Si no detecta ningun objeto:
                    else:
                        #Detengo el motor
                        bridge1.write(0)
                        bridge2.write(0)
            cv2.imshow("mask image", mask) # Muestro la imagen de la mascara
            cv2.imshow("window", video) # Muestro la imagen de la webcam
            if cv2.waitKey(1) & 0xFF == ord("s"):
                break
    webcam_video.release()
    cv2.destroyAllWindows()
        
        
if __name__=="__main__":
    asyncio.run(main())
    