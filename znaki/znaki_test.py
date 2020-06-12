import cv2 as cv
import numpy as np
import dlib
import os

#cap = cv.VideoCapture(0)

sezd = cv.imread("validation/sezd.png")
parking = cv.imread("validation/parking.jpg")
forvard = cv.imread("validation/forvard.png")
right = cv.imread("validation/right.jpg")
left = cv.imread("validation/left.jpg")
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


while(True):
    cv.imshow("sezd", sezd)
    cv.imshow("forvard", forvard)
    cv.imshow("right", right)
    cv.imshow("left", left)
    cv.imshow("parking", parking)
    cv.imshow("m_sezd", mask_sezd)
    cv.imshow("m_forvard", mask_forvard)
    cv.imshow("m_right", mask_right)
    cv.imshow("m_left", mask_left)
    cv.imshow("m_parking", mask_parking)

    if cv.waitKey(1) == ord("q"):
        break
#cap.release()
cv.destroyAllWindows()