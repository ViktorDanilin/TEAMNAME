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
    binaryg = cv.inRange(gray, 0, 100)

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
KD = 0
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

TRAP = np.float32([[80, 299],
                   [270, 299],
                   [235, 250],
                   [115, 250]])
TRAPINT = np.array(TRAP, dtype=np.int32)

cap = cv2.VideoCapture(0)
ch_povor = 1

povors = [False] * 40

s = serial.Serial('/dev/ttyS3', 115200, timeout=1)
# s = None

while True:
    ret, frame = cap.read()

    img = cv2.resize(frame, SIZE)
    binary = binarize(img)

    for e in TRAP:
        for i in TRAP:
            cv2.line(img, tuple(e), tuple(i), (255, 0, 0), 2)

    perspective = trans_perspective(binary, TRAP, RECT, SIZE)
    edges = cv2.Canny(perspective, 100, 200)

    and_line = np.array([0] * 400)
    for line in perspective[:10]:
        and_line = np.bitwise_or(and_line, line)

    last = 0
    kol = 0
    for e in and_line:
        if e > 0 and last == 0:
            kol += 1
        last = e

    if kol > 1:
        # print('povor', ch_povor)
        povors.append(True)
        povors.pop(0)
        ch_povor += 1
    else:
        povors.append(False)
        povors.pop(0)

    if povors.count(True) > 15:
        print('povors', ch_povor)
        start_t = time()
        while time() - start_t < 3:
            command(s, 65, 1, 10)
            sleep(0.05)
        break

    grad = (find_left_right(perspective, 0))
    command(s, grad, 1, 10)

    #cv2.imshow('pers', perspective)
    #cv2.imshow('im', img)
    stream(edges)

    sleep(0.05)
    q = cv2.waitKey(10)
    if q == ord('q'):
        break

