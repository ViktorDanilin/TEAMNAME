import numpy as np
import json
from matplotlib import pyplot as plt
from skimage import color
from skimage.feature import hog
from sklearn import svm
from sklearn.metrics import classification_report,accuracy_score
import cv2
from os import listdir
import pickle

DIR = 'images'

images = []
labels = []
for folder in listdir(DIR):
    for file in listdir(DIR + '/' + folder):
        images.append(cv2.imread(DIR + '/' + folder + '/' + file))
        labels.append(folder)

data_gray = [ cv2.cvtColor(cv2.resize(i, (100, 100)), cv2.COLOR_BGR2GRAY) for i in images ]

ppc = 16
hog_images = []
hog_features = []
for image in data_gray:
    fd,hog_image = hog(image, orientations=8, pixels_per_cell=(ppc,ppc),cells_per_block=(4, 4),block_norm= 'L2',visualize=True)
    hog_images.append(hog_image)
    hog_features.append(fd)

labels =  np.array(labels).reshape(len(labels),1)

clf = svm.SVC()
hog_features = np.array(hog_features)
data_frame = np.hstack((hog_features,labels))
np.random.shuffle(data_frame)

#What percentage of data you want to keep for training
percentage = 80
partition = int(len(hog_features)*percentage/100)

x_train, x_test = data_frame[:partition,:-1],  data_frame[partition:,:-1]
y_train, y_test = data_frame[:partition,-1:].ravel() , data_frame[partition:,-1:].ravel()

clf.fit(x_train,y_train)

with open('clf.pkl', 'wb') as f:
    pickle.dump(clf, f)

y_pred = clf.predict(x_test)

print(y_test)
print(y_pred)
print("Accuracy: "+str(accuracy_score(y_test, y_pred)))
print('\n')
print(classification_report(y_test, y_pred))