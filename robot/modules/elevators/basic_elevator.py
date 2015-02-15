import asyncio
import yeti
import wpilib

from yeti.wpilib_extensions import Referee
from yeti.interfaces import gamemode

class BasicElevator(yeti.Module):

    def module_init(self):
        self.referee = Referee(self)

        self.lift_jag = wpilib.CANJaguar(10)
        self.lift_jag.setVoltageMode()
        self.referee.watch(self.lift_jag)

        self.joystick = wpilib.Joystick(1)
        self.referee.watch(self.joystick)

    @asyncio.coroutine
    @gamemode.teleop_task
    def manual_elevator(self):
        self.lift_jag.enableControl()
        while gamemode.is_teleop():
            value = self.joystick.getRawAxis(1) * 12
            self.lift_jag.set(value)
            yield from asyncio.sleep(.05)
        self.lift_jag.disableControl()