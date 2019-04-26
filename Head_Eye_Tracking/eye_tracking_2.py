#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 23 22:52:43 2019

@author: jagtarsingh
"""

#from PIL import Image, ImageDraw
import face_recognition
import cv2
import time
import pdb
import serial
# ser = serial.Serial(
#     port='/dev/cu.usbmodem54269801',
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
    ser.port = '/dev/cu.usbmodem54269801'
    ser.timeout = 5
    ser.open()
    print(datetime.now())

# Load the jpg file into a numpy array
video_capture = cv2.VideoCapture(0)

process_this_frame = True

while True:
    t1 = time.time()
    # Grab a single frame of video
    ret, frame = video_capture.read()
    
    if ret:
    # Find all facial features in all the faces in the image
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = small_frame[:, :, ::-1]
        
        if process_this_frame:
            face_landmarks_list = face_recognition.face_landmarks(small_frame)


        process_this_frame = not process_this_frame
        
        for face_landmarks in face_landmarks_list:
        
            # Print the location of each facial feature in this image
            coord_right = face_landmarks['right_eye']
            x_right = int(sum(list(map(lambda x: x[0], coord_right)))/len(coord_right))
            y_right = int(sum(list(map(lambda x: x[1], coord_right)))/len(coord_right))
            coord_left = face_landmarks['left_eye']
            x_left = int(sum(list(map(lambda x: x[0], coord_left)))/len(coord_left))
            y_left = int(sum(list(map(lambda x: x[1], coord_left)))/len(coord_left))

            x_send = int(4*x_right*254/1280)
            y_send = int(4*y_right*254/920)
            # print(x_send)
            # ser.write(str.encode('x'))
            # ser.write(str.encode(chr(x_send)))
            # ser.write(str.encode('y'))
            # ser.write(str.encode(chr(y_send)))
            MESSAGE = [str(x_send).zfill(3),str(y_send).zfill(3),'\n']
            print(MESSAGE)
            # history.append(MESSAGE)
            MESSAGE = ''.join(MESSAGE)
            if SERIAL:
                ser.write(MESSAGE.encode("utf-8"))

            # ser.write(str.encode(chr(y_send)))
        #    pdb.set_trace()
        #    for facial_feature in face_landmarks.keys():
        #
        #        print("The {} in this face has the following points: {}".format(facial_feature, face_landmarks[facial_feature]))
            # line = [(4*x_right,4*y_right),(4*x_left,4*y_left)]
            # print(line)
            # if line[0][0]>640:
            #     print("on the left")
            # else:
            #     print("on the right")
            # Let's trace out each facial feature in the image with a line!
        #    for facial_feature in face_landmarks.keys():
#            cv2.line(frame,(4*x_right,4*y_right),(4*x_left,4*y_left),(255,0,0),5)
            cv2.line(frame,(x_right,y_right),(x_left,y_left),(255,0,0),5)
        
        # # Display the resulting image
        cv2.imshow('Video', small_frame)

        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # t2 = time.time()
    # if t2 != t1:
    #     print("%f FPS" % (1/(t2-t1)))
    # else:
    #     print("timeout")

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()


# def mapPixel2angle(pixel):
    
#     if pixel>640 and pixel<740:
#         return 15#degrees
    
    
    
    
#     return angle

