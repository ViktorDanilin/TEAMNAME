from helpers import constrain


class Motors:
    def __init__(self, interface=None):
        self.interface = interface

    def command(self, angle, dir, speed):
        comm = "SPD {},{},{},{} ".format(constrain(int(angle), 65, 125), dir, speed, 0)
        # print(comm)
        if self.interface is not None:
            self.interface.write(comm.encode())

    def stop(self):
        self.command(90, 1, 0)
