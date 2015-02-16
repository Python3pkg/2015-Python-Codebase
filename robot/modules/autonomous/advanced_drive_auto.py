import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams, remote_coroutines

class AdvancecdDriveAuto(yeti.Module):
    """A basic autonomous routine that simply moves forward for a given speed and time"""

    def module_init(self):
        self.drivetrain_control = datastreams.get_datastream("auto_drive_setpoint")

    @asyncio.coroutine
    @gamemode.autonomous_task
    def do_auto(self):
        # Pause for a moment to ensure other systems have initialized.
        yield from asyncio.sleep(.1)
        self.logger.info("Starting autonomous mode given by module " + self.name)
        self.drivetrain_control.push({"y_pos": 10})
        yield from remote_coroutines.call_public_coroutine("auto_drive_enable")
        yield from asyncio.sleep(5)
        #yield from remote_coroutines.call_public_coroutine("auto_drive_disable")
        self.logger.info("Ending autonomous mode -- waiting for mode to change")
        while gamemode.is_autonomous():
            yield from asyncio.sleep(.5)
        self.logger.info("Ended autonomous mode")
