import cv2
import zmq
import numpy as np
from threading import Thread
from base64 import b64decode
import pygame
import sys
from control import Motors
from serial import Serial

pygame.init()
screen = pygame.display.set_mode([400, 300])
pygame.display.set_caption('rc car inno')

forward = 0
turn = 90
sp = 25

m = Motors(Serial('/dev/ttyS3', 115200, timeout=1))

class Viewer:
    def __init__(self, callback, port=5555):
        self.port = port
        self.callback = callback
        self.context = zmq.Context()
        self.footage_socket = self.context.socket(zmq.SUB)
        self.footage_socket.bind('tcp://*:{}'.format(self.port))
        self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

        self.thread = Thread(target=self._threaded_func)
        self.thread.start()

    def _threaded_func(self):
        while True:
            window, frame = self.footage_socket.recv_string().split(';')
            npimg = np.fromstring(b64decode(frame), dtype=np.uint8)
            img = cv2.imdecode(npimg, 1)
            self.callback(window, img)

def f(window, frame):
    global keys
    while True:
        screen.fill([0, 0, 0])
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = pygame.surfarray.make_surface(frame)
        screen.blit(frame, (0, 0))
        pygame.display.update()
        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == None:
                sys.exit(0)


def controlling():
    global forward, turn, sp
    while True:
        if keys[pygame.K_w]:
            forward = True
        if keys[pygame.K_a]:
            turn -= 35
        if keys[pygame.K_d]:
            turn += 35
        if keys[pygame.K_UP]:
            sp -= 10
        if keys[pygame.K_DOWN]:
            sp += 10

        if forward == True:
            if sp < 25:
                sp = 25
            elif sp > 55:
                sp = 55
            m.command(turn, 1, sp)
        else:
            m.stop()

        forward = False
        turn = 90


if __name__ == '__main__':
    Viewer(f, 5555)
    t = Thread(target=controlling())
    t.start()