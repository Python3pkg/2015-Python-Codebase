import asyncio
import wpilib
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.wpilib_extensions import Referee

class PWMMecanum(yeti.Module):
    """
    A bare-bones, simplistic mecanum drive utilizing RobotDrive.
    """

    #Values to convert from fps to percentage
    MAX_X_FPS = 14
    MAX_Y_FPS = 14
    MAX_R_RPS = 2

    def module_init(self):
        #Initialize the Referee for the module.
        self.referee = Referee(self)

        #Setup robot drive
        self.robotdrive = wpilib.RobotDrive(0, 1, 2, 3)
        self.referee.watch(self.robotdrive)

        #Start control datastream
        #Values are:
        # forward_fps -- desired forward speed in feet-per-second
        # right_fps -- desired right strafe speed in feet-per-second
        # clockwise_rps -- desired clockwise rotational speed in rotations-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")



    @gamemode.enabled_task
    @asyncio.coroutine
    def drive_loop(self):

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_speed = control_data.get("forward_fps", 0)
            right_speed = control_data.get("right_fps", 0)
            clockwise_speed = control_data.get("clockwise_rps", 0)

            #Convert measurements to throttle percentages.
            forward_percentage = forward_speed / self.MAX_Y_FPS
            right_percentage = right_speed / self.MAX_X_FPS
            clockwise_percentage = clockwise_speed / self.MAX_R_RPS

            #Drive the motors.
            self.robotdrive.mecanumDrive_Cartesian(right_percentage, forward_percentage, clockwise_percentage, 0)

            #Pause for a moment to let the rest of the code run
            yield from asyncio.sleep(.05)

