import asyncio
import yeti
import wpilib
from robotpy_ext.common_drivers.xl_max_sonar_ez import MaxSonarEZPulseWidth

from yeti.wpilib_extensions import Referee


class UltrasonicTest(yeti.Module):

    def module_init(self):
        self.referee = Referee(self)

        self.ultrasonic = MaxSonarEZPulseWidth(0)
        self.referee.watch(self.ultrasonic)
        #wpilib.LiveWindow.addSensor(self.name, "ultrasonic", self.ultrasonic)

        self.testtalon = wpilib.Talon(0)
        self.referee.watch(self.testtalon)
        #wpilib.LiveWindow.addActuator(self.name, "Talon", self.testtalon)

        self.din = wpilib.DigitalInput(2)
        self.referee.watch(self.din)
        #wpilib.LiveWindow.addActuator(self.name, "DIn", self.din)

    @asyncio.coroutine
    @yeti.autorun_coroutine
    def run_loop(self):
        wpilib.SmartDashboard.putNumber("range", self.ultrasonic.get())
