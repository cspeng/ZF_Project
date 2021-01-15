#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 28 02:57:19 2015
@author: PUPPY
"""
import cv2

cascfeature= 'haarcascade_eye.xml'
eyeCascade= cv2.CascadeClassifier(cascfeature)
capture= cv2.VideoCapture(0)
capture.set(3,1024)
capture.set(4,768)
font= cv2.FONT_HERSHEY_SIMPLEX; red= (0,164,0)

while(True):
    grabbed, frame = capture.read()
    if grabbed==True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        '''
        eye = eyeCascade.detectMultiScale(gray, 1.2, 15,minSize=(35,35))
        for (x, y, w, h) in eye:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,160,128), 2)
        '''
        cv2.putText(frame, "press x to exit", (200,470), font, 1, red, 2)
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('x'):
            break
    if capture.isOpened()==False:
        break
capture.release()
cv2.destroyAllWindows()
