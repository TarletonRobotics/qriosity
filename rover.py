import gps
from pyfirmata import ArduinoMega, util
import time


class Rover(object):
    """Create a rover object based on a pin map
    """
    def __init__(self, port, forward, reverse):
        """Return new rover object.
        analog pins will be pin numbers w/o A
        """
        self.port = port
        #self.f = forward
        #self.r = reverse
        self.currentPwm = 0;
        self.startTime = 0;
        self.currentTime = 0;
        self.flippedDirection = False;
        self.board = ArduinoMega(self.port)
        #self.gps = GpsDevice(self)

    def delay(self, length):
        start = time.time()
        while(time.time() - start < length):
            pass

    def setDriveSpeed(self, speed):
        if speed > 0:
            self.MAN1.write(1.0);
            self.MAN2.write(0.0);
        else:
            self.MAN1.write(0.0);
            self.MAN2.write(1.0);

    # def readBattery(self):
    #     rawVoltage = self.BAT.read()
    #     # convert voltage to percentage
    #     self.batPerc = rawVoltage / 1023.0

    def drive(self, speed):
        if speed == 0:
            self.MAN1.write(0.0)
            self.MAN2.write(0.0)
            return

        if not self.flippedDirection and speed * self.currentPwm < 0:
            self.startTime = time.time()
            self.flippedDirection = True
            self.MAN1.write(0.0)
            self.MAN2.write(0.0)

        self.currentTime = time.time()

        if not self.flippedDirection:
            self.setDriveSpeed(speed)
            self.currentPwm = speed
        elif self.currentTime - self.startTime > 1:
            self.setDriveSpeed(speed)
            self.currentPwm = speed
            self.flippedDirection = False

    def turn(self, direction):
        if direction > 0:
            self.SDIG1.write(0)
            self.SDIG2.write(0)
            self.SAN1.write(1)
            self.SAN2.write(1)
        elif direction < 0:
            self.SDIG1.write(1)
            self.SDIG2.write(1)
            self.SAN1.write(1)
            self.SAN2.write(1)

        self.delay(0.1)
        self.SAN1.write(0)
        self.SAN2.write(0)

    def set_port_pins(self):
        """
        the firmata pin methods take a string
        """
        # forward = 'd:'+str(labrat.f)+':p'
        # reverse = 'd:'+str(labrat.r)+':p'
        # self.pinForward = board.get_pin(forward)
        # self.pinReverse = board.get_pin(reverse)

        self.MAN1 = self.board.get_pin("d:3:p")
        self.MAN2 = self.board.get_pin("d:5:p")
        self.SAN1 = self.board.get_pin("d:6:p")
        self.SAN2 = self.board.get_pin("d:2:p")
        self.SDIG1 = self.board.get_pin("d:8:o")
        self.SDIG2 = self.board.get_pin("d:4:o")

        # battery read
        #self.BAT = self.board.get_pin("a:97:i")

        #self.pause = board.pass_time(5)