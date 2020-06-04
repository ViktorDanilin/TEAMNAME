import cv2
import cv2 as cv
import numpy as np
from time import sleep, time
import serial

import base64
import zmq

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
    binaryh = cv.inRange(hls, (0, 0, 0), (255, 255, 100))

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    binaryg = cv.inRange(gray, 0, 50)

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
KP = 0.17
KI = 0.1
KD = 0.4
last = 0
def find_left_right(perspective, d=0):
    global integral, last, KP, KI, KD

    hist = np.sum(perspective[perspective.shape[0] // 2:, :], axis=0)
    left = np.argmax(hist)

    if d:
        cv.line(perspective, (left, 0), (left, 300), 50, 2)

        cv.imshow('lines', perspective)

    err = -1 * (left - perspective.shape[1] // 2)

    pid = KP * err + KD * (err - last) + KI * integral
    last = err
    integral += err
    integral = constrain(integral, -10, 10)

    return 90 + pid


def find_left_right_canny(edges, d=0):
    global integral, last, KP, KI, KD

    first = edges[-1]
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

    sred = (left + right) // 2
    cv.line(edges, (right, 0), (right, 300), 50, 10)
    # stream(edges)
    err = -1 * (right - first.shape[0] // 2)

    pid = KP * err + KD * (err - last) + KI * integral
    last = err
    integral += err
    integral = constrain(integral, -10, 10)

    return 90 + pid

def command(s ,angle, dir, speed):
    comm = "SPD {},{},{} ".format(constrain(int(angle), 65, 125), dir, speed)
    print(comm)
    if s is not None:
        s.write(comm.encode())

# constants
SIZE = (400, 300)

RECT = np.float32([[0, 299],
                   [399, 299],
                   [399, 0],
                   [0, 0]])

TRAP = np.float32([[100, 299],
                   [310, 299],
                   [275, 250],
                   [135, 250]])
TRAPINT = np.array(TRAP, dtype=np.int32)

cap = cv2.VideoCapture(0)
ch_povor = 1

povors = [False] * 40

# s = serial.Serial('/dev/ttyS3', 115200, timeout=1)
s = None

while True:
    ret, frame = cap.read()

    img = cv2.resize(frame, SIZE)
    binary = binarize(img)

    for e in TRAP:
        for i in TRAP:
            cv2.line(img, tuple(e), tuple(i), (255, 0, 0), 2)

    perspective = trans_perspective(binary, TRAP, RECT, SIZE)
    edges = cv2.Canny(perspective, 100, 200)

    grad = (find_left_right_canny(edges, 0))
    command(s, grad, 1, 20)

    #cv2.imshow('pers', perspective)
    #cv2.imshow('im', img)
    stream(img)

    sleep(0.05)
    q = cv2.waitKey(10)
    if q == ord('q'):
        break

