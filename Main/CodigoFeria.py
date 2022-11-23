import cv2
from pyfirmata import Arduino, SERVO, util
import time
import asyncio
import numpy as np

def main():
    arduino=Arduino("COM9")
    arduino.digital[9].mode=SERVO
    arduino.digital[9].write(90)


    webcam_video=cv2.VideoCapture(1)
    lowerColor = np.array([ 0, 155, 165])
    upperColor = np.array([30, 255, 255])


    while True:
        success, video, = webcam_video.read()
        if success==True:
            
            video=cv2.resize(video,(600,600))
            img=cv2.GaussianBlur(video,(11,11),10)
            #video=cv2.line(video, (300,0), (300,600), (255, 0, 0), 3)
            video=cv2.flip(video, 1)
            #video=cv2.flip(video, 0)
            #video = video[100:500, 20:550]
            video = cv2.normalize(video, None, alpha=0,beta=350, norm_type=cv2.NORM_MINMAX)
            img=cv2.cvtColor(video, cv2.COLOR_BGR2HSV)
            mask=cv2.inRange(img, lowerColor, upperColor)
            mask_contours, hierarchy=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(mask_contours)!=0:
                for mask_contour in mask_contours:
                    if cv2.contourArea(mask_contour) > 400 <800: #Si el area es mayor a 500, entonces se dibuja el contorno
                        x, y, w, h = cv2.boundingRect(mask_contour)
                        cv2.rectangle(video, (x, y), (x + w, y + h), (0, 0, 255), 3)
                        cnt=mask_contours
                        M = cv2.moments(mask_contour)
                        if (M["m00"]==0): M["m00"]=1
                        x = int(M["m10"]/M["m00"])
                        y = int(M['m01']/M['m00'])
                        cv2.circle(video, (x, y), 7, (255, 255, 255), -1)
                        cv2.putText(video, f"x:{x} y:{y}", (x - 20, y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (39 , 236, 230), 2)
                        #print(y)

                        errorX=x-300

                        OldMax=300
                        OldMin=-300
                        NewMax=165
                        NewMin=0

                        OldRange = (OldMax - OldMin)  
                        NewRange = (NewMax - NewMin)  
                        NewValue = (((errorX - OldMin) * NewRange) / OldRange) + NewMin                

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
                        print( int(NewValue))
                        arduino.digital[9].write(int(NewValue))
                     
            
            cv2.imshow("mask image", mask) # Muestro la imagen de la mascara
            cv2.imshow("window", video) # Muestro la imagen de la webcam
            if cv2.waitKey(1) & 0xFF == ord("s"):
                break
    webcam_video.release()
    cv2.destroyAllWindows()
if __name__=="__main__":
    main()