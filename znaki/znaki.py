import cv2 as cv
import numpy as np
import time

video = 'videos/outpy1.avi'
cap = cv.VideoCapture(video)

sezd = cv.imread("images/sezd.png")
parking = cv.imread("images/parking.png")
forvard = cv.imread("images/forvard.png")
right = cv.imread("images/right.png")
left = cv.imread("images/left.png")
sezd = cv.resize(sezd, (64, 64))
forvard = cv.resize(forvard, (64, 64))
right = cv.resize(right, (64, 64))
left = cv.resize (left, (64, 64))
parking = cv.resize(parking, (64, 64))
mask_sezd = cv.inRange(sezd, (10, 17, 20), (255, 255, 65))
mask_forvard = cv.inRange(forvard, (10, 17, 20), (255, 255, 65))
mask_right = cv.inRange(right, (10, 17, 20), (255, 255, 65))
mask_left = cv.inRange(left, (10, 17, 20), (255, 255, 65))
mask_parking = cv.inRange(parking, (10, 17, 20), (255, 255, 65))
mask_sezd = cv.dilate(mask_sezd,None,iterations=1)
mask_sezd = cv.erode(mask_sezd,None,iterations=1)
mask_forvard = cv.dilate(mask_forvard,None,iterations=1)
mask_forvard = cv.erode(mask_forvard,None,iterations=1)
mask_right = cv.dilate(mask_right,None,iterations=1)
mask_right = cv.erode(mask_right,None,iterations=1)
mask_left = cv.dilate(mask_left,None,iterations=1)
mask_left = cv.erode(mask_left,None,iterations=1)
mask_parking = cv.dilate(mask_parking,None,iterations=1)
mask_parking = cv.erode(mask_parking,None,iterations=1)

while (cap.isOpened()):
    time.sleep(0.1)
    ret, frame = cap.read()
    if ret == True:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv = cv.blur(hsv, (5, 5))

        mask = cv.inRange(hsv,(105,105,45),(255,255,110))

        contours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        contours = contours[0]
        if contours:
            contours = sorted(contours, key=cv.contourArea, reverse=True)
            cv.drawContours(frame, contours, 0, (0, 0, 225), 2)
            (x, y, w, h) = cv.boundingRect(contours[0])
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 225, 0), 2)
            roImg = frame[y:y + h, x:x + w]
            roImg = cv.resize(roImg, (64, 64))
            roImg = cv.inRange(roImg, (105,105,45),(255,255,110))
            cv.imshow("frame",frame)
            cv.imshow("res", roImg)
            forvard_val = 0
            right_val = 0
            left_val = 0
            for i in range(64):
                for j in range(64):
                    if roImg[i][j] == mask_forvard[i][j]:
                        forvard_val = forvard_val + 1
                    if roImg[i][j] == mask_right[i][j]:
                        right_val = right_val + 1
                    if roImg[i][j] == mask_left[i][j]:
                        left_val = left_val + 1
            if (forvard_val > 2300):
                print("forvard")
            if (right_val > 2300):
                print("right")
            if (left_val > 2500):
                print("left")
            else:
                print("nothing")
        if cv.waitKey(1) == ord("q"):
            break
cap.release()
cv.destroyAllWindows()