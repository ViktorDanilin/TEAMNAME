import cv2
import dlib
import time
model_detector = dlib.simple_object_detector("tld_forward.swm")

video = 'videos/outpy.avi'
cap = cv2.VideoCapture(video)

while (True):
    ret, frame = cap.read()

    boxes = model_detector(frame)
    for box in boxes:
        print("True")
        (x, y, xb, yb) = [box.left(), box.top(), box.right(), box.bottom()]
        cv2.rectangle(frame, (x, y), (xb, yb), (0, 0, 225), 2)

    cv2.imshow("frame", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()