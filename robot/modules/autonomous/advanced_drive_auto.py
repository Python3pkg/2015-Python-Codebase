import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams, remote_methods

class AdvancecdDriveAuto(yeti.Module):
    """A basic autonomous routine that simply moves forward for a given speed and time"""

    def module_init(self):
        self.drivetrain_control = datastreams.get_datastream("auto_drive_setpoint")

    @asyncio.coroutine
    @gamemode.autonomous_task
    def do_auto(self):
        self.logger.info("Starting autonomous mode given by module " + self.name)
        remote_methods.call_public_method("auto_drive_reset_tracking")
        self.drivetrain_control.push({"r_pos": 90})
        remote_methods.call_public_method("auto_drive_enable")
        yield from remote_methods.call_public_coroutine("auto_drive_wait_for_y")
        yield from remote_methods.call_public_coroutine("auto_drive_disable")
        self.logger.info("Ending autonomous mode -- waiting for mode to change")
        while gamemode.is_autonomous():
            yield from asyncio.sleep(.5)
        self.logger.info("Ended autonomous mode")
