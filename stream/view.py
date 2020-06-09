import cv2
import zmq
from base64 import b64decode
import numpy as np
from threading import Thread


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
    vi = cv2.resize(frame, (640, 480))
    cv2.imshow(window, vi)
    cv2.waitKey(1)


v = Viewer(f, 5555)
v.thread.join()
