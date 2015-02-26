import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine

class AdvancecdDriveAuto(yeti.Module):
    """A basic autonomous routine that simply moves forward for a given speed and time"""

    def module_init(self):
        self.drivetrain_control = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_config = datastreams.get_datastream("drivetrain_auto_config")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")


    @asyncio.coroutine
    @gamemode.autonomous_task
    def do_auto(self):
        self.logger.info("Starting autonomous mode given by module " + self.name)
        start_time = wpilib.Timer.getFPGATimestamp()

        call_public_method("drivetrain.reset_sensor_input")
        call_public_method("drivetrain.auto_drive_enable")

        self.drivetrain_control.push({"x_pos": 8})
        yield from call_public_coroutine("drivetrain.wait_for_x")

        call_public_method("drivetrain.auto_drive_disable")
        self.logger.info("Ending autonomous mode -- waiting for mode to change")
        self.logger.info("Autonomous routine took {} seconds total".format(wpilib.Timer.getFPGATimestamp() - start_time))
        while gamemode.is_autonomous():
            yield from asyncio.sleep(.5)
        self.logger.info("Ended autonomous mode")
