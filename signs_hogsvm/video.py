from skimage.feature import hog
import skimage
import cv2
import pickle

with open('clf.pkl', 'rb') as f:
    clf = pickle.load(f)

cap = cv2.VideoCapture(0)

print(skimage.__version__)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame[100:200, 100:200], cv2.COLOR_BGR2GRAY)
    ppc = 16
    fd, hog_image = hog(gray, orientations=8, pixels_per_cell=(ppc, ppc), cells_per_block=(4, 4), block_norm='L2',
                        visualize=True)
    cv2.imshow('hog', hog_image)
    clas = clf.predict([fd])
    cv2.putText(gray, clas[0], (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
    print(clas)
    cv2.imshow('gray', gray)
    cv2.waitKey(1)
