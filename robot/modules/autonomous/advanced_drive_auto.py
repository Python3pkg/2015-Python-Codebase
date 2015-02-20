import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine

class AdvancecdDriveAuto(yeti.Module):
    """A basic autonomous routine that simply moves forward for a given speed and time"""

    def module_init(self):
        self.drivetrain_control = datastreams.get_datastream("drivetrain_auto_setpoint")

    @asyncio.coroutine
    @gamemode.autonomous_task
    def do_auto(self):
        self.logger.info("Starting autonomous mode given by module " + self.name)
        call_public_method("drivetrain.reset_sensor_input")
        self.drivetrain_control.push({"r_pos": 90})
        call_public_method("drivetrain.auto_drive_enable")
        yield from call_public_coroutine("drivetrain.wait_for_r")
        call_public_method("drivetrain.auto_drive_disable")
        self.logger.info("Ending autonomous mode -- waiting for mode to change")
        while gamemode.is_autonomous():
            yield from asyncio.sleep(.5)
        self.logger.info("Ended autonomous mode")
