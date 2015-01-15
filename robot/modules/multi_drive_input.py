import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams


class MultiDriveInput(yeti.Module):

    MODE_TOGGLE = True
    #Input modes:
    #0: Tank drive
    #1: Toggle drive
    #2: Three Axis drive (If Enabled)
    DEFAULT_MODE = 0
    THREE_AXIS_MODE = False

    #Maximum values for input loop to output
    MAX_Y_INPUT_FPS = 14
    MAX_X_INPUT_FPS = 14
    MAX_ROT_INPUT_DPS = 2

    #Square the normalized outputs to provide a logarithmic scale effect.
    SQUARE_OUTPUTS = False

    def module_init(self):
        joytick_ids = [0, 1]
        self.joysticks = [wpilib.Joystick(i) for i in joytick_ids]

        #Start control datastream
        #Values are:
        # forward_fps -- desired forward speed in feet-per-second
        # right_fps -- desired right strafe speed in feet-per-second
        # clockwise_dps -- desired clockwise rotational speed in degrees-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")

    @gamemode.teleop_task
    @asyncio.coroutine
    def input_loop(self):
        last_mode_toggle = True
        input_mode = self.DEFAULT_MODE

        #Loop until end of teleop mode.
        while gamemode.is_teleop():

            #Select input mode
            if self.MODE_TOGGLE:
                #On rising edge
                if not last_mode_toggle and self.joysticks[0].getRawButton(9):
                    input_mode += 1

                    if input_mode > 2 or (not self.THREE_AXIS_MODE and input_mode > 1):
                        input_mode = 0

                    if input_mode == 0:
                        self.logger.info("Switched to Tank Drive")
                    elif input_mode == 1:
                        self.logger.info("Switched to Toggle Drive")
                    elif input_mode == 2:
                        self.logger.info("Switched to 3-Axis Drive")

                last_mode_toggle = self.joysticks[0].getRawButton(9)

            #############################################################################
            # Because the drive_loop expects real-world measurements for desired motion, we
            # first calculate the desired values as percentage of max speed, Then
            # scale the values to get real-world measurements
            #

            #Motion values from -1 to 1
            forward_percentage = 0
            right_percentage = 0
            clockwise_percentage = 0

            if input_mode == 0:
                ly = -self.joysticks[0].getY()
                lx = self.joysticks[0].getX()
                ry = -self.joysticks[1].getY()
                rx = self.joysticks[1].getX()

                forward_percentage = (ly + ry)/2
                right_percentage = (lx + rx)/2
                clockwise_percentage = (ly - ry)/2

            elif input_mode == 1:
                strafemode = self.joysticks[0].getRawButton(8)

                if strafemode:
                    forward_percentage = -self.joysticks[0].getY()
                    right_percentage = -self.joysticks[0].getX()
                else:
                    forward_percentage = -self.joysticks[0].getY()
                    clockwise_percentage = -self.joysticks[0].getX()

            elif input_mode == 2:
                if not self.THREE_AXIS_MODE:
                    input_mode = 0

            #If enabled, square the outputs to provide a logarithmic effect.
            if self.SQUARE_OUTPUTS:
                forward_percentage **= 2
                right_percentage **= 2
                clockwise_percentage **= 2

            #Scale to real-world measurments
            forward_fps = forward_percentage * self.MAX_Y_INPUT_FPS
            right_fps = right_percentage * self.MAX_X_INPUT_FPS
            clockwise_fps = clockwise_percentage * self.MAX_ROT_INPUT_DPS

            #Send values to drive module
            self.control_datastream.push({"forward_fps": forward_fps, "right_fps": right_fps, "clockwise_rps": clockwise_fps})

            #Pause for a moment to let the rest of the code run.
            yield from asyncio.sleep(.05)
