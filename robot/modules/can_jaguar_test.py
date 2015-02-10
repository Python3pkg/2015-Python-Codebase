import asyncio
import yeti
import wpilib

from yeti.wpilib_extensions import Referee
from yeti.interfaces import gamemode

class CanJagTest(yeti.Module):

    def module_init(self):
        self.referee = Referee(self)

        self.jstick = wpilib.Joystick(0)

        self.canjag = wpilib.CANJaguar(14)
        self.referee.watch(self.canjag)

        #wpilib.LiveWindow.addActuator("jagtest", "jag10", self.canjag)
        #wpilib.LiveWindow.setEnabled(True)
        #self.canjag.setPositionModeQuadEncoder(10, 2, 2, 0)
        #self.canjag.setPercentMode()
        self.canjag.setCurrentModePID(1, .1, 0)
        self.canjag.enableControl()

        self.start_coroutine(self.status_loop())

    @asyncio.coroutine
    def status_loop(self):
        while True:
            self.canjag.verify()
            wpilib.SmartDashboard.putNumber("CanJagMode", self.canjag.getControlMode())
            wpilib.SmartDashboard.putNumber("CanJagGet", self.canjag.get())
            wpilib.SmartDashboard.putNumber("CanJagVolts", self.canjag.getOutputVoltage())
            wpilib.SmartDashboard.putNumber("CanJagCurrent", self.canjag.getOutputCurrent())
            wpilib.SmartDashboard.putNumber("CanJagEncoder", self.canjag.getPosition())
            yield from asyncio.sleep(.1)

    @gamemode.teleop_task
    @asyncio.coroutine
    def cycle_loop(self):
        while gamemode.is_teleop():
            val = self.jstick.getRawAxis(1)*60
            self.canjag.set(val)
            wpilib.SmartDashboard.putNumber("output", val)
            yield from asyncio.sleep(.05)