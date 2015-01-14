__author__ = 'Tim'
import asyncio
import wpilib
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.wpilib_extensions import Referee

class MecanumMultiDrive(yeti.Module):
    """
    A 1-2 Joystick Mecanum drivetrain module, pressing button 10 to switch between Tank and Toggle drive.
    When using Toggle drive, button 9 switches between turn and strafe.
    """

    USE_CAN = False
    USE_ENCODERS = False

    USE_INPUT_MODE_TOGGLE = True
    #Input modes:
    #0: Tank drive
    #1: Toggle drive
    #2: Three Axis drive (If Enabled)
    DEFAULT_INPUT_MODE = 0
    THREE_AXIS_MODE = False

    #Maximum values for input loop to output
    MAX_Y_INPUT_FPS = 10
    MAX_X_INPUT_FPS = 10
    MAX_ROT_INPUT_DPS = 300

    P = 1
    I = 2
    D = 1

    def module_init(self):
        #Initialize the Referee for the module.
        self.referee = Referee(self)

        #Setup the array of control joysticks
        joystick_ids = range(0, 3)
        self.joysticks = list()
        for joystick_id in joystick_ids:
            self.joysticks.append(wpilib.Joystick(joystick_id))

        #Setup motor controllers

        #Motors are always used in lists in the following order:
        #Front Left
        #Rear Left
        #Front Right
        #Rear Right
        if self.USE_CAN:
            motor_ids = [10, 11, 12, 13]
        else:
            motor_ids = [0, 1, 2, 3]

        self.motor_controllers = list()
        for motor_id in motor_ids:
            if self.USE_CAN:
                #CAN Bus: Init and Configure the Jaguar
                controller = wpilib.CANJaguar(motor_id)
                if self.USE_ENCODERS:
                    controller.setSpeedModeQuadEncoder(360, self.P, self.I, self.D)
                else:
                    controller.setPercentMode()
            else:
                #PWM: Just init the jaguar
                controller = wpilib.Jaguar(motor_id)
            self.referee.watch(controller)
            self.motor_controllers.append(controller)

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
        input_mode = self.DEFAULT_INPUT_MODE

        #Loop until end of teleop mode.
        while gamemode.is_teleop():

            #Select input mode
            if self.USE_INPUT_MODE_TOGGLE:
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


            #Scale to real-world measurments

            forward_fps = forward_percentage * self.MAX_Y_INPUT_FPS
            right_fps = right_percentage * self.MAX_X_INPUT_FPS
            clockwise_fps = clockwise_percentage * self.MAX_ROT_INPUT_DPS

            #Save values for drive loop

            self.control_datastream.push({"forward_fps": forward_fps, "right_fps": right_fps, "clockwise_fps": clockwise_fps})

            #Pause for a moment to let the rest of the code run.
            yield from asyncio.sleep(.05)

    @gamemode.enabled_task
    @asyncio.coroutine
    def drive_loop(self):

        if self.USE_CAN:
            for controller in self.motor_controllers:
                controller.enableControl()

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_speed = control_data.get("forward_fps", 0)
            right_speed = control_data.get("right_fps", 0)
            clockwise_speed = control_data.get("clockwise_fps", 0)

            #Scale control inputs if we aren't using can + encoders
            if not (self.USE_CAN and self.USE_ENCODERS):
                forward_speed /= self.MAX_Y_INPUT_FPS
                right_speed /= self.MAX_X_INPUT_FPS
                clockwise_speed /= self.MAX_ROT_INPUT_DPS

            #Inverse kinematics to get mecanum values
            front_left_out = forward_speed + clockwise_speed + right_speed
            front_right_out = forward_speed - clockwise_speed - right_speed
            rear_left_out = forward_speed + clockwise_speed - right_speed
            rear_right_out = forward_speed - clockwise_speed + right_speed

            #Normalize values if we aren't using can + encoders
            if not (self.USE_CAN and self.USE_ENCODERS):
                max_value = max(abs(front_left_out), abs(front_right_out), abs(rear_left_out), abs(rear_right_out))
                if max_value > 1:
                    front_left_out /= max_value
                    front_right_out /= max_value
                    rear_left_out /= max_value
                    rear_right_out /= max_value

            #Send to motors.
            self.motor_controllers[0].set(front_left_out)
            self.motor_controllers[1].set(rear_left_out)
            self.motor_controllers[2].set(front_right_out)
            self.motor_controllers[3].set(rear_right_out)

            #Pause for a moment to let the rest of the code run
            yield from asyncio.sleep(.05)

        if self.USE_CAN:
            for controller in self.motor_controllers:
                controller.disableControl()
