# OpenCV program to detect face in real time 
# import libraries of python OpenCV  
# where its functionality resides 
import cv2  
import pdb
import serial
#import pyaudio
import sys
import numpy as np
#import aubio
# load the required trained XML classifiers 
# https://github.com/Itseez/opencv/blob/master/ 
# data/haarcascades/haarcascade_frontalface_default.xml 
# Trained XML classifiers describes some features of some 
# object we want to detect a cascade function is trained 
# from a lot of positive(faces) and negative(non-faces) 
# images. 
face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml') 
#person_cascade = cv2.CascadeClassifier(
#    './haarcascade_upperbody.xml')
# https://github.com/Itseez/opencv/blob/master 
# /data/haarcascades/haarcascade_eye.xml 
# Trained XML file for detecting eyes 
#eye_cascade = cv2.CascadeClassifier('./haarcascade_eye.xml')  

# ser = serial.Serial(
#     port='/dev/cu.SLAB_USBtoUART',
#     baudrate=115200,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS,
#     timeout=0
# )
SERIAL = False
if SERIAL:
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = '/dev/cu.SLAB_USBtoUART'
    ser.timeout = 5
    ser.open()
    # print(datetime.now())


# capture frames from a camera 
cap = cv2.VideoCapture(0) 
  
# loop runs if capturing has been initialized. 
while 1:  
  
    # reads frames from a camera 
    ret, img = cap.read()  

    if ret:
#        pdb.set_trace()
        # convert to gray scale of each frames 
        # print(img.shape)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
      
        # Detects faces of different sizes in the input image 
        faces = face_cascade.detectMultiScale(gray, 1.3, 6) 
#        people = person_cascade.detectMultiScale(gray) 
        small_frame = cv2.resize(img, (0, 0), fx=0.25, fy=0.25)
        
        ##
        if len(faces)>0:
            max_area = 0
            i = 0
            max_ind = 0
            ###########
            for (x,y,w,h) in faces:
                area = w*h
                if area>max_area:
                    max_area = area
                    max_ind = i
                i+=1
            
            ##########
            x,y,w,h = faces[max_ind]
            center_x = int(x+w/2)
            center_y = int(y+h/2)

            MESSAGE = [str(center_x).zfill(4),str(center_y).zfill(4),'\n']
            print(MESSAGE)
            # history.append(MESSAGE)
            MESSAGE = ''.join(MESSAGE)
            if SERIAL:
                ser.write(MESSAGE.encode("utf-8"))

            # #######Earllier Code Neck####
            # if center_x>600 and center_x<680:
            #     send_x = 127
            #     print(127)
            # else:
            #     send_x = int(center_x*254/1280)
            #     print(send_x)
                
            # center_y = y+h/2
            # if center_y>420 and center_y<480:
            #     send_y = 127
            #     print(127)
            # else:
            #     send_y = int(center_y*254/920)
            #     print(send_y)
            
            # ser.write(str.encode(chr(send_x)))
            # ser.write(str.encode(chr(send_y)))
#################################
        
#        if len(people)>0:
#            max_area = 0
#            i = 0
#            max_ind = 0
#            ###########
#            for (x,y,w,h) in people:
#                area = w*h
#                if area>max_area:
#                    max_area = area
#                    max_ind = i
#                i+=1
#            
#            ##########
#            x,y,w,h = people[max_ind]
#            center = x+w/2
#            if center>600 and center<680:
#                ser.write(str.encode(chr(127)))
#                print(127)
#            else:
#                out = int(x*255/1280)
#                print(out)
#                ser.write(str.encode(chr(out)))
#        
        ###########
        
        for (x,y,w,h) in faces:
#            area = w*h
#            if area>max_area:
#                max_area = area
#                max_ind = i
#            i+=1
            
            
            x=int(x*0.25)
            y=int(y*0.25)
            w=int(w*0.25)
            h=int(h*0.25)
            # To draw a rectangle in a face  
            cv2.rectangle(small_frame,(x,y),(x+w,y+h),(255,255,0),2)  
            roi_gray = gray[y:y+h, x:x+w] 
            roi_color = img[y:y+h, x:x+w]
        
#        for (x,y,w,h) in people:
##            area = w*h
##            if area>max_area:
##                max_area = area
##                max_ind = i
##            i+=1
#            
#            
#            x=int(x*0.25)
#            y=int(y*0.25)
#            w=int(w*0.25)
#            h=int(h*0.25)
#            # To draw a rectangle in a face  
#            cv2.rectangle(small_frame,(x,y),(x+w,y+h),(255,255,0),2)  
#            roi_gray = gray[y:y+h, x:x+w] 
#            roi_color = img[y:y+h, x:x+w]
      
            # Detects eyes of different sizes in the input image 
#            eyes = eye_cascade.detectMultiScale(roi_gray)  
      
            #To draw a rectangle in eyes 
#            for (ex,ey,ew,eh) in eyes: 
#                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,127,255),2) 
        
        # Display an image in a window 
        cv2.imshow('img',small_frame) 
      
        # Wait for Esc key to stop 
        k = cv2.waitKey(30) & 0xff
        if k == 27: 
            break
    
  
# Close the window 
cap.release() 
  
# De-allocate any associated memory usage 
cv2.destroyAllWindows()