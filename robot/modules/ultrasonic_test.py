import asyncio
import yeti
import wpilib
from robotpy_ext.common_drivers.xl_max_sonar_ez import MaxSonarEZPulseWidth

from yeti.wpilib_extensions import Referee

class UltrasonicTest(yeti.Module):

    def module_init(self):
        self.ultrasonic = MaxSonarEZPulseWidth(0)

    @asyncio.coroutine
    @yeti.autorun
    def enabled(self):
        while True:
            wpilib.SmartDashboard.putNumber("range", self.ultrasonic.get())
            yield from asyncio.sleep(1)

    def module_deinit(self):
        self.ultrasonic.free()

