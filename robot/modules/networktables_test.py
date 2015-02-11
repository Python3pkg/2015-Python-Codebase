import yeti
import wpilib
import asyncio
from yeti.wpilib_extensions import Referee

class NetworktablesTest(yeti.Module):

    def module_init(self):
        self.referee = Referee(self)
        self.m1 = wpilib.Talon(0)
        self.referee.watch(self.m1)
        self.e1 = wpilib.Encoder(0, 1)
        self.referee.watch(self.e1)
        self.d1 = wpilib.DigitalInput(2)
        self.referee.watch(self.d1)
        self.g1 = wpilib.Gyro(0)
        self.referee.watch(self.g1)
        self.c1 = wpilib.Counter(3)
        #wpilib.LiveWindow.setEnabled(True)

    @asyncio.coroutine
    @yeti.autorun_coroutine
    def ticker(self):
        i = 0
        while True:
            i += .2
            if i > 1:
                i = 0
            #self.m1.set(i)
            wpilib.SmartDashboard.putNumber("number", i)
            print(i)
            yield from asyncio.sleep(1)

    def module_deinit(self):
        wpilib.LiveWindow.setEnabled(False)