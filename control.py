from helpers import constrain
from time import sleep


class Motors:
    frame_ids = {'reboot': 0}

    def __init__(self, interface=None):
        self.interface = interface
        sleep(0.5)
        self.stop()
        sleep(0.5)
        self.frame_ids['start'] = self.read('reboot')

    def command(self, angle, dir, speed):
        comm = "SPD {},{},{},{} ".format(constrain(int(angle), 65, 125), dir, speed, 0)
        # print(comm)
        if self.interface is not None:
            self.interface.write(comm.encode())

    def stop(self):
        self.command(90, 1, 0)

    def read(self, frame_id='start'):
        if self.interface is not None:
            s = self.interface.readline().replace(b'\n', b'').decode()
            if s[0] == 'F' and s[-1] == 'E':
                return float(s[1:-1]) - self.frame_ids[frame_id]
        else:
            return 0
