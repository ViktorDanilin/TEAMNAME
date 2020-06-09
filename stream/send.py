import cv2
from base64 import b64encode
import zmq


class Streamer:
    resolution = (200, 150)

    def __init__(self, ip='localhost', port=5555):
        self.ip = ip
        self.port = port
        self.context = zmq.Context()
        self.footage_socket = self.context.socket(zmq.PUB)
        self.footage_socket.connect('tcp://{}:{}'.format(self.ip, self.port))

    def stream(self, wnidow_name, frame):
        resized = cv2.resize(frame, self.resolution)  # resize the frame
        _, buffer = cv2.imencode('.jpg', resized)
        base64_image = b64encode(buffer).decode()
        self.footage_socket.send_string(wnidow_name + ';' + base64_image)


# # USAGE
# s = Streamer('localhost', 5555)
#
# cap = cv2.VideoCapture(0)
# while True:
#     _, frame = cap.read()
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     s.stream('color', frame)
#     s.stream('gray', gray)
#
#     cv2.waitKey(1)