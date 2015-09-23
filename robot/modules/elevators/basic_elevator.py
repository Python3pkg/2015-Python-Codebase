import asyncio
import yeti
import wpilib

class BasicElevator(yeti.Module):

    def module_init(self):
        self.lift_jag = wpilib.CANJaguar(10)
        self.lift_jag.setVoltageMode()
        self.joystick = wpilib.Joystick(1)

    def teleop_init(self):
        self.lift_jag.enableControl()

    def teleop_periodic(self):
        value = self.joystick.getRawAxis(1) * -12
        self.lift_jag.set(value)

    def disabled_init(self):
        self.lift_jag.disableControl()

    def module_deinit(self):
        self.lift_jag.free()
