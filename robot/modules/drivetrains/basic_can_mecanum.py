import asyncio
import yeti
import wpilib

from yeti.wpilib_extensions import Referee
from yeti.interfaces import gamemode, datastreams


#Helper funcs
def threshold_value(self, value, threshold):
    if abs(value) <= threshold:
        value = 0
    return value


def signing_square(self, value):
    if value < 0:
        return value ** 2
    else:
        return -(value ** 2)


class BasicCANMecanum(yeti.Module):
    """
    A basic mecanum controller, taking input from the 'drivetrain_control' datastream
    and outputting to CAN Jaguars in percent mode.
    """

    #CAN IDS for the Drivetrain Jaguars in the following order:
    #Front Left
    #Rear Left
    #Front Right
    #Rear Right
    CAN_IDS = [13, 11, 12, 10]

    ####################################
    # JOYSTICK CONTROLLER CONF

    #Values for scaling joystick input

    MAX_Y_INPUT = 1
    MAX_X_INPUT = 1
    MAX_ROT_INPUT = 1

    SQUARE_INPUTS = True



    def module_init(self):
        #Initialize the Referee for the module.
        self.referee = Referee(self)

        #Setup motor controllers

        self.motor_controllers = list()
        for motor_id in self.CAN_IDS:
            controller = wpilib.CANJaguar(motor_id)
            controller.setPercentMode()

            self.referee.watch(controller)
            self.motor_controllers.append(controller)

        self.joystick = wpilib.Joystick(0)

        #Start control datastream
        #Values are:
        # forward_fps -- desired forward speed in feet-per-second
        # right_fps -- desired right strafe speed in feet-per-second
        # clockwise_dps -- desired clockwise rotational speed in degrees-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")

    @gamemode.teleop_task
    @asyncio.coroutine
    def joystick_loop(self):
        while gamemode.is_teleop():
            forward_percentage = -self.joystick.getY()
            right_percentage = self.joystick.getX()
            clockwise_percentage = self.joystick.getZ()

            #Threshold values
            forward_percentage = threshold_value(forward_percentage, .10)
            right_percentage = threshold_value(right_percentage, .10)
            clockwise_percentage = threshold_value(clockwise_percentage, .15)

            if self.SQUARE_INPUTS:
                forward_percentage = signing_square(forward_percentage)
                right_percentage = signing_square(right_percentage)
                clockwise_percentage = signing_square(clockwise_percentage)

            #Scale to real-world measurements
            forward_percentage *= self.MAX_Y_INPUT
            right_percentage *= self.MAX_X_INPUT
            clockwise_percentage *= self.MAX_ROT_INPUT

            #Send values to drive loop
            self.control_datastream.push({"forward_percentage": forward_percentage, "right_percentage": right_percentage, "clockwise_percentage": clockwise_percentage})

            yield from asyncio.sleep(.05)

    @gamemode.enabled_task
    @asyncio.coroutine
    def drive_loop(self):

        #Clear datastream
        self.control_datastream.push({"forward_percentage": 0, "right_percentage": 0, "clockwise_percentage": 0})

        #Enable the jaguars
        for controller in self.motor_controllers:
            controller.enableControl()

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_percentage = control_data.get("forward_percentage", 0)
            right_percentage = control_data.get("right_percentage", 0)
            clockwise_percentage = control_data.get("clockwise_percentage", 0)

            #Inverse kinematics to get mecanum values
            front_left_out = forward_percentage + clockwise_percentage + right_percentage
            front_right_out = forward_percentage - clockwise_percentage - right_percentage
            rear_left_out = forward_percentage + clockwise_percentage - right_percentage
            rear_right_out = forward_percentage - clockwise_percentage + right_percentage

            #Normalize values if we aren't using can + encoders
            max_value = max(abs(front_left_out), abs(front_right_out), abs(rear_left_out), abs(rear_right_out))
            if max_value > 1:
                front_left_out /= max_value
                front_right_out /= max_value
                rear_left_out /= max_value
                rear_right_out /= max_value

            #Send to motors.
            self.motor_controllers[0].set(front_left_out)
            self.motor_controllers[1].set(rear_left_out)
            self.motor_controllers[2].set(-front_right_out)
            self.motor_controllers[3].set(-rear_right_out)

            #Pause for a moment to let the rest of the code run
            yield from asyncio.sleep(.05)

        #Disable the jaguars
        for controller in self.motor_controllers:
            controller.disableControl()

    def module_deinit(self):
        #Disable the jaguars
        for controller in self.motor_controllers:
            controller.disableControl()
