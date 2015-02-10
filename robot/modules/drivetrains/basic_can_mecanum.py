import asyncio
import yeti
import wpilib

from yeti.wpilib_extensions import Referee
from yeti.interfaces import gamemode, datastreams

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

    #Values to convert from fps to percentage
    MAX_X_FPS = 14
    MAX_Y_FPS = 14
    MAX_R_RPS = 1

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

        #Start control datastream
        #Values are:
        # forward_fps -- desired forward speed in feet-per-second
        # right_fps -- desired right strafe speed in feet-per-second
        # clockwise_rps -- desired clockwise rotational speed in rotations-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")



    @gamemode.enabled_task
    @asyncio.coroutine
    def drive_loop(self):

        #Enable the jaguars
        for controller in self.motor_controllers:
            controller.enableControl()

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_speed = control_data.get("forward_fps", 0)
            right_speed = control_data.get("right_fps", 0)
            clockwise_speed = control_data.get("clockwise_rps", 0)

            wpilib.SmartDashboard.putNumber("forward_fps", forward_speed)
            wpilib.SmartDashboard.putNumber("right_fps", right_speed)
            wpilib.SmartDashboard.putNumber("clockwise_speed", clockwise_speed)

            forward_percentage = forward_speed / self.MAX_Y_FPS
            right_percentage = right_speed / self.MAX_X_FPS
            clockwise_percentage = clockwise_speed / self.MAX_R_RPS

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

            wpilib.SmartDashboard.putNumber("front_left_out", front_left_out)
            wpilib.SmartDashboard.putNumber("front_right_out", front_right_out)
            wpilib.SmartDashboard.putNumber("rear_left_out", rear_left_out)
            wpilib.SmartDashboard.putNumber("rear_right_out", rear_right_out)


            #Pause for a moment to let the rest of the code run
            yield from asyncio.sleep(.05)

        #Disable the jaguars
        for controller in self.motor_controllers:
            controller.disableControl()

    def module_deinit(self):
        #Disable the jaguars
        for controller in self.motor_controllers:
            controller.disableControl()
