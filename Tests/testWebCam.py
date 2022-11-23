import cv2

#Importo configuracion de lectura de rostros y lo guardo en variable face_cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
#Indico que camara realizará la lectura (WebCam)
capture = cv2.VideoCapture(1)

#Ciclo de funcionamiento infinito
while True:
    
    _, img = capture.read()
    escale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(escale, 1.1, 4)
    for(x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
    cv2.imshow('img', img)
    esc = cv2.waitKey(30)
    if esc == 27:
        break
capture.release()