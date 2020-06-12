import cv2
import dlib
import os
import xml.etree.ElementTree as pars

dir = r"/home/viktor/RRO_2020/TEAMNAME/znaki/"
images = []
annots = []

imgNameList = os.listdir(dir + "images/left/")

print(imgNameList)

for FileName in imgNameList:
    image = cv2.imread(dir + "images/left/" + FileName)
    #print(dir + "images" + FileName)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    OnlyFileName = FileName.split(".")[0]
    print(OnlyFileName)
    e = pars.parse(dir + "annotations/left/" + OnlyFileName + ".xml")
    root = e.getroot()
    object = root.find("object")
    object = object.find("bndbox")
    x = int(object.find("xmin").text)
    y = int(object.find("ymin").text)
    x2 = int(object.find("xmax").text)
    y2 = int(object.find("ymax").text)

    images.append(image)
    annots.append([dlib.rectangle(left=x, top=y, right=x2, bottom=y2)])

options = dlib.simple_object_detector_training_options()
options.be_verbose = True

detector = dlib.train_simple_object_detector(images, annots, options)

detector.save("tld_forward.swm")
print("Detector saved")