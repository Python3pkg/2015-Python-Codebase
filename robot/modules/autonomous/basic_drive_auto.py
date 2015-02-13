import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams

class BasicDriveAuto(yeti.Module):
    """A basic autonomous routine that simply moves forward for a given speed and time"""

    def module_init(self):
        self.drivetrain_control = datastreams.get_datastream("drivetrain_control")

    @asyncio.coroutine
    @gamemode.autonomous_task
    def do_auto(self):
        # Pause for a moment to ensure other systems have initialized.
        yield from asyncio.sleep(.1)
        self.logger.info("Starting autonomous mode given by module " + self.name)
        self.drivetrain_control.push({"forward_fps": 2})
        yield from asyncio.sleep(5)
        self.drivetrain_control.push({"forward_fps": 0})
        self.logger.info("Ending autonomous mode -- waiting for mode to change")
        while gamemode.is_autonomous():
            yield from asyncio.sleep(.5)
        self.logger.info("Ended autonomous mode")
