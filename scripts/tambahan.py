#!/usr/bin/python
import numpy as np
import cv2

frame,thresh,image_hsv = None, None, None
inputMode = False
mouseY, mouseX = None,None
roi = None

def nothing(angka):
    pass

def selectROI(event, x, y, flags, param):
    global new_img, mouseX, mouseY, inputMode, image_hsv, roi
    if inputMode and event == cv2.EVENT_LBUTTONDOWN:
        # cv2.circle(new_img, (x,y), 4, (0,255,0), 2)
        mouseX, mouseY = x,y
        roi = image_hsv[mouseY-10:mouseY+10, mouseX-10:mouseX+10]
        
cv2.namedWindow("clahe")
cv2.setMouseCallback("clahe", selectROI)

vid = cv2.VideoCapture(0)

while True:
    success, img = vid.read()

    if not success:
        break

    new_img = img.copy()

    image_hsv = cv2.cvtColor(new_img, cv2.COLOR_BGR2HSV)

    if mouseX is not None:
        hue = np.mean(roi[:,:, 0])
        sat = np.mean(roi[:,:, 1])
        val = np.mean(roi[:,:, 2])
        h_low = hue - 25
        h_high = hue + 25
        s_low = sat - 25
        s_high = sat + 25
        v_low = val - 25
        v_high = val + 25

        mask1 = cv2.inRange(image_hsv, (h_low, s_low, v_low), (h_high, s_high, v_high))
        ret,thresh = cv2.threshold(mask1, 40, 255, 0)

        _, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) != 0:
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(new_img, (x, y), (x + w, y + h), (0, 255,0), 2)
        
        cv2.imshow("mask", thresh)

    cv2.imshow("clahe", new_img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("i"):
        inputMode = True

    cv2.waitKey(30)
