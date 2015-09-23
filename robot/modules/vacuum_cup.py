import yeti
import asyncio
import wpilib

class VacuumCup(yeti.Module):

    def module_init(self):
        self.joystick = wpilib.Joystick(0)
        self.pump_relay = wpilib.Relay(0)
        self.vent_solenoid = wpilib.Solenoid(0)
        self.gameclock = self.engine.get_module("gameclock")

    @asyncio.coroutine
    def enabled(self):
        while self.gameclock.is_enabled():
            pump_button = self.joystick.getRawButton(1)
            if pump_button:
                self.pump_relay.set(wpilib.Relay.Value.kForward)
                self.vent_solenoid.set(False)
            else:
                self.pump_relay.set(wpilib.Relay.Value.kOff)
                self.vent_solenoid.set(True)
            yield from asyncio.sleep(.05)
        self.pump_relay.set(wpilib.Relay.Value.kOff)