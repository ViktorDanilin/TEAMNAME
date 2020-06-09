import cv2
import cv2 as cv
import numpy as np
from time import sleep, time
import serial
import sys

import base64
import zmq

from simple_pid import PID
pid = PID(0.2, 0, 0.1, setpoint=150)
pid_right = PID(0.23, 0, 0.2, setpoint=250)

context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.connect('tcp://46.161.156.148:5555')


def stream(frame):
    frame1 = cv2.resize(frame, (200, 150))  # resize the frame
    encoded, buffer = cv2.imencode('.jpg', frame1)
    jpg_as_text = base64.b64encode(buffer)
    footage_socket.send(jpg_as_text)


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


def trans_perspective(binary, trap, rect, size, d=0):
    M = cv.getPerspectiveTransform(trap, rect)
    perspective = cv.warpPerspective(binary, M, size, flags=cv.INTER_LINEAR)
    if d:
        cv.imshow('perspective', perspective)
    return perspective


def constrain(val, minv, maxv):
    return min(maxv, max(minv, val))


integral = 0
KP = 0.2
KI = 0
KD = 0.3
last = 0
def find_left_right(perspective, d=0):
    global integral, last, KP, KI, KD

    if False and timeout_detect_stop > 0 and int(time()) < timeout_detect_stop + 5:
        print('canny')
        return find_left_right_canny(perspective)
    else:
        hist = np.sum(perspective[perspective.shape[0] // 2:, :], axis=0)
        left = np.argmax(hist)

        if d:
            cv.line(perspective, (left, 0), (left, 300), 50, 2)

            cv.imshow('lines', perspective)

        control = pid(left)

        # err = -1 * (left - perspective.shape[1] // 2)
        #
        # pid = KP * err + KD * (err - last) + KI * integral
        # last = err
        # integral += err
        # integral = constrain(integral, -10, 10)

        return 90 + control


def find_left_right_canny(edges, d=0):
    global integral, last, KP, KI, KD

    # first = edges[100]
    first = np.sum(perspective[200:250], axis=0)
    # print(first.shape)

    left = 200
    for i, e in enumerate(first):
        if e > 0:
            left = i
            break

    right = 200
    for i, e in enumerate(first[::-1]):
        if e > 0:
            right = first.shape[0] - i
            break

    # if timeout_detect_stop > 0 and int(time()) < timeout_detect_stop + 2:
    #     if right > 200:
    #         print('sl')
    #         right = max(left, 200)

    sred = (left + right) // 2
    cv.line(edges, (right, 0), (right, 300), 50, 10)
    cv.line(edges, (left, 0), (left, 300), 50, 10)
    # stream(edges)
    control = pid_right(right)
    print(right)

    return 90 + control

def command(s ,angle, dir, speed):
    comm = "SPD {},{},{},{} ".format(constrain(int(angle), 65, 125), dir, speed, 0)
    # print(comm)
    if s is not None:
        s.write(comm.encode())

# TODO: to one func
def check_cnt(c):
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.1 * peri, True)
    if len(approx) == 4:
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)
        print(ar)
        if 0.4 < ar < 0.7:
            return True
    return False


def findBigContour(mask):
    contours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)[0]
    if contours:
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        for c in contours:
            if 1000 < cv.contourArea(c) < 40000:
                if check_cnt(c):
                    # print(c)
                    return c
    else:
        return None


timeout_detect_stop = -100
def detect_stop(perspective):
    global timeout_detect_stop
    if int(time()) > timeout_detect_stop + 10:
        stoplin = 0
        for i in range(250, 299):
            stoplin += int(np.sum(perspective[i, :], axis=0) // 255)
        print(stoplin)
        if stoplin > 7000:
            timeout_detect_stop = int(time())
            # print('stop linia')
            return True
        else:
            return False
    else:
        return False


last_tl = ''
tl_allowed = True
def standardize_input(image):
    global last_tl, tl_allowed
    frame = cv2.resize(image, (44, 81))
    # frame = cv2.flip(frame, 1)
    cut = frame
    hsv = cv2.cvtColor(cut, cv2.COLOR_BGR2GRAY)
    v = cv2.inRange(hsv, 100, 255)

    red_s = np.sum(v[0:27, 0:44])
    yellow_s = np.sum(v[28:54, 0:44])
    green_s = np.sum(v[55:81, 0:44])
    yellow_red_s = np.sum(v[0:54, 0:44])

    cv2.rectangle(cut, (0, 0), (44, 27), (0, 0, 255), 2)
    cv2.rectangle(cut, (0, 28), (44, 54), (0, 255, 255), 2)
    cv2.rectangle(cut, (0, 55), (44, 81), (0, 255, 0), 2)

    standard_im = cut

    print(red_s, yellow_s, green_s)
    if red_s < 5000 and yellow_s < 5000 and green_s < 5000:
        print('blink')
        last_tl = 'off'
        tl_allowed = True
    else:
        if green_s > red_s-5000 and green_s > yellow_s-5000:
            if last_tl == 'off':
                print('blink')
                tl_allowed = True
            else:
                print('green')
                last_tl = 'green'
                tl_allowed = True
        elif red_s > green_s or yellow_s > green_s:
            if red_s > yellow_s+10000:
                print('red')
            elif yellow_s > red_s+10000:
                print('yellow')
            elif yellow_red_s > 10000:
                print('yellow and red')

            last_tl = 'red'
            tl_allowed = False

    return standard_im


# constants
SIZE = (400, 300)

RECT = np.float32([[0, 299],
                   [399, 299],
                   [399, 0],
                   [0, 0]])

TRAP = np.float32([[100, 299],
                   [300, 299],
                   [275, 240],
                   [125, 240]])
TRAPINT = np.array(TRAP, dtype=np.int32)

cap = cv2.VideoCapture(0)
ch_povor = 1

out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, SIZE)


povors = [False] * 40

s = serial.Serial('/dev/ttyS3', 115200, timeout=1)
# s = None

command(s, 90, 1, 0)
sleep(1)
krasniy = False
# timeout_detect_stop = int(time())
while True:
    ret, frame = cap.read()

    img = cv2.resize(frame, SIZE)
    binary = binarize(img)

    tl = img[img.shape[0]//3:int(img.shape[0]/1.3),img.shape[1]//2:]
    binary_tl = binarize(tl)

    for e in TRAP:
        for i in TRAP:
            cv2.line(img, tuple(e), tuple(i), (255, 0, 0), 2)

    perspective = trans_perspective(binary, TRAP, RECT, SIZE)
    edges = cv2.Canny(perspective, 100, 200)

    grad = (find_left_right(edges, 0))


    # if detect_stop(perspective):
    #     print('stop')
    #     # command(s, grad, 1, 0)
    #     # break

    cnt = findBigContour(binary_tl)

    if cnt is not None:
        (x, y, w, h) = cv.boundingRect(cnt)

        # if cnt is not None:
        # cv.drawContours(tl, cnt, -1, (255, 0, 0), 10)
        stream(tl[y:y+h, x:x+w])
        standardize_input(tl[y:y+h, x:x+w])
        # command(s, 90, 1, 0)
    else:
        #cv2.imshow('pers', perspective)
        #cv2.imshow('im', img)
        stream(img)

    if detect_stop(perspective):
        if cnt is not None:
            if not tl_allowed:
                print('stop')
                command(s, grad, 1, 0)
                krasniy = True

    if krasniy:
        print('krs')
        command(s, grad, 1, 0)
        if tl_allowed:
            krasniy = False
    else:
        command(s, grad, 1, 15)


    # stream(img)
    q = cv2.waitKey(10)
    if q == ord('q'):
        break

s.close()