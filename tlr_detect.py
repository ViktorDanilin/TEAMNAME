import cv2
import cv2 as cv
import numpy as np
from time import sleep


def binarize(img, d=0):
    hls = cv.cvtColor(img, cv.COLOR_BGR2HLS)
    binaryh = cv.inRange(hls, (0, 0, 0), (255, 255, 255))

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    binaryg = cv.inRange(gray, 0, 40)

    binary = cv.bitwise_and(binaryg, binaryh)

    if d:
        cv.imshow('hls', binaryh)
        cv.imshow('gray', binaryg)
        cv.imshow('bin', binary)
    return binary


def check_cnt(c):
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.1 * peri, True)
    if len(approx) == 4:
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)
        print(ar)
        if 0.4 < ar < 0.6:
            return True
    return False

def findBigContour(mask):
    contours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)[0]
    if contours:
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        for c in contours:
            if 1000 < cv.contourArea(c) < 10000:
                if check_cnt(c):
                    return c
    else:
        return None


cap = cv2.VideoCapture('trl.mp4')

while True:
    ret, frame = cap.read()
    binary = binarize(frame[frame.shape[0]//3:int(frame.shape[0]/1.3),frame.shape[1]//2:])

    cnt = findBigContour(binary)

    cv.drawContours(binary, cnt, -1, 50, 10)

    cv2.imshow('im', binary)

    sleep(0.05)

    cv2.waitKey(1)