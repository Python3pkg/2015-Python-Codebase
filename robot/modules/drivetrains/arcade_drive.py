import asyncio
import wpilib
import yeti

from yeti.interfaces import gamemode
from yeti.wpilib_extensions import Referee


class ArcadeDrive(yeti.Module):
    """
    A bare-bones example of an arcade drive module.
    """

    def module_init(self):
        #Initialize the Referee for the module.
        self.referee = Referee(self)

        #Setup a joystick
        self.joystick = wpilib.Joystick(0)
        self.referee.watch(self.joystick)

        #Setup the robotdrive
        self.robotdrive = wpilib.RobotDrive(0, 1, 2, 3)
        self.referee.watch(self.robotdrive)

    @gamemode.teleop_task
    @asyncio.coroutine
    def teleop_loop(self):

        #Loop until end of teleop mode.
        while gamemode.is_teleop():

            #Get the joystick values and drive the motors.
            self.robotdrive.arcadeDrive(-self.joystick.getY(), -self.joystick.getX())

            #Pause for a moment to let the rest of the code run.
            yield from asyncio.sleep(.05)
