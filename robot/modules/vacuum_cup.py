import yeti
import asyncio
import wpilib
from yeti.interfaces import gamemode

class VacuumCup(yeti.Module):

    def module_init(self):
        self.joystick = wpilib.Joystick(0)
        self.pump_relay = wpilib.Relay(0)
        self.vent_solenoid = wpilib.Solenoid(0)

    @asyncio.coroutine
    @gamemode.enabled_task
    def run_pump(self):
        print("Entering")
        while gamemode.is_enabled():
            pump_button = self.joystick.getRawButton(1)
            if pump_button:
                self.pump_relay.set(wpilib.Relay.Value.kForward)
                self.vent_solenoid.set(False)
            else:
                self.pump_relay.set(wpilib.Relay.Value.kOff)
                self.vent_solenoid.set(True)
            yield from asyncio.sleep(.05)
        self.pump_relay.set(wpilib.Relay.Value.kOff)