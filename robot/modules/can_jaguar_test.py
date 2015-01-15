import asyncio
import yeti
import wpilib

from yeti.wpilib_extensions import Referee
from yeti.interfaces import gamemode

class CanJagTest(yeti.Module):

    def module_init(self):
        self.referee = Referee(self)

        self.canjag = wpilib.CANJaguar(10)
        self.referee.watch(self.canjag)
        self.canjag.setVoltageMode()
        self.canjag.enableControl()

    @gamemode.teleop_task
    @asyncio.coroutine
    def cycle_loop(self):
        while gamemode.is_teleop():
            self.canjag.set(12)
            self.canjag.verify()
            self.logger.info(str(self.canjag.getControlMode()))
            self.logger.info(str(self.canjag.getOutputVoltage()))
            self.logger.info(str(self.canjag.getOutputCurrent()))
            self.logger.info(str(self.canjag.get()))
            yield from asyncio.sleep(2)