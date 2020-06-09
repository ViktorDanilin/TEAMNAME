import cv2
import time
def nothing(x):
    pass

video = 'videos/outpy1.avi'

cap=cv2.VideoCapture(video)

cv2.namedWindow("result")
cv2.createTrackbar("minb","result",0,255,nothing)
cv2.createTrackbar("ming","result",0,255,nothing)
cv2.createTrackbar("minr","result",0,255,nothing)

cv2.createTrackbar("maxb","result",0,255,nothing)
cv2.createTrackbar("maxg","result",0,255,nothing)
cv2.createTrackbar("maxr","result",0,255,nothing)


while(cap.isOpened()): #True

  ret,frame=cap.read()
  if ret == True:
    time.sleep(0.1)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",hsv)
    minb = cv2.getTrackbarPos("minb","result")
    ming = cv2.getTrackbarPos("ming", "result")
    minr = cv2.getTrackbarPos("minr", "result")

    maxb = cv2.getTrackbarPos("maxb", "result")
    maxg = cv2.getTrackbarPos("maxg", "result")
    maxr = cv2.getTrackbarPos("maxr", "result")



    mask = cv2.inRange(hsv,(105,145,45),(255,255,110))
    cv2.imshow("mask",mask)           
    result = cv2.bitwise_and(frame,frame, mask = mask)
    cv2.imshow("result",result)
    if cv2.waitKey(1)==ord("q"):
        break
  else:
    break

cap.release()
cv2.destroyAllWindows()