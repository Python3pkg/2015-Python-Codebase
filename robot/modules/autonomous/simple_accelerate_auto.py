import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams

class SimpleAccelerateAuto(yeti.Module):
    """A basic autonomous routine that simply accelerates forward for a given and time"""

    def module_init(self):
        self.drivetrain_control = datastreams.get_datastream("drivetrain_control")

    @asyncio.coroutine
    @gamemode.autonomous_task
    def do_auto(self):
        # Pause for a moment to ensure other systems have initialized.
        yield from asyncio.sleep(.1)
        accel = 6
        speed = 0
        while speed < 5:
            yield from asyncio.sleep(.05)
            speed += accel * .05
            self.drivetrain_control.push({"forward_fps": speed})
        yield from asyncio.sleep(2.5)
        while speed > 1:
            yield from asyncio.sleep(.05)
            speed -= accel * .05
            self.drivetrain_control.push({"forward_fps": speed})
        self.drivetrain_control.push({"forward_fps": 0})
        self.logger.info("Ending autonomous mode -- waiting for mode to change")
        while gamemode.is_autonomous():
            yield from asyncio.sleep(.5)
        self.logger.info("Ended autonomous mode")

