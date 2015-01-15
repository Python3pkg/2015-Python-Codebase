import asyncio
import wpilib
import yeti
import numpy as np
import time

from yeti.interfaces import gamemode, datastreams
from yeti.wpilib_extensions import Referee

class AdvancedCANMecanum(yeti.Module):

    #CAN IDS for the Drivetrain Jaguars in the following order:
    #Front Left
    #Rear Left
    #Front Right
    #Rear Right
    #CAN_IDS = [10, 11, 12, 13]
    CAN_IDS = [0, 1, 2, 3]

    #Jagur PID Values
    JAG_P = 1
    JAG_I = 2
    JAG_D = .1

    #Gyro PID Values
    GYRO_P = 1
    GYRO_I = 2
    GYRO_D = .1

    #Matrix to mix forward, right, clockwise values into mecanum wheel values
    MECHANUM_MATRIX = np.mat([[+1, +1, +1],
                              [+1, -1, +1],
                              [+1, +1, -1],
                              [+1, -1, -1]])

    ENCODER_TICKS_PER_ROTATION = 360

    def module_init(self):
        #Initialize the Referee for the module.
        self.referee = Referee(self)

        #Setup motor controllers
        self.motor_controllers = list()
        for motor_id in self.CAN_IDS:
            controller = wpilib.Jaguar(motor_id)
            #controller = wpilib.CANJaguar(motor_id)
            #controller.setSpeedModeQuadEncoder(self.ENCODER_TICKS_PER_ROTATION, self.JAG_P, self.JAG_I, self.JAG_D)

            self.referee.watch(controller)
            self.motor_controllers.append(controller)

        #setup gyro
        self.gyro = wpilib.Gyro(0)

        #Setup rotational PID controller
        self.rot_pid = wpilib.PIDController(self.GYRO_P, self.GYRO_I, self.GYRO_D, self.get_rot_pid_val, self.save_rot_pid_out)

        #Start control datastream
        #Values are:
        # forward_fps -- desired forward speed in feet-per-second
        # right_fps -- desired right strafe speed in feet-per-second
        # clockwise_rps -- desired clockwise rotational speed in rotations-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")

    pid_out = 0

    def save_rot_pid_out(self, value):
        self.pid_out = value

    def get_rot_pid_val(self):
        return self.gyro.getRate()

    @gamemode.enabled_task
    @asyncio.coroutine
    def drive_loop(self):
        self.rot_pid.enable()

        #for controller in self.motor_controllers:
        #    controller.enableControl()

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_speed = control_data.get("forward_fps", 0)
            right_speed = control_data.get("right_fps", 0)
            clockwise_speed = control_data.get("clockwise_rps", 0)

            #Init output matrix
            output = np.mat([[0],
                             [0],
                             [0],
                             [0]])

            #Handle rotation PID first
            self.rot_pid.setSetpoint(clockwise_speed)
            clockwise_val = self.pid_out

            #Package control values into matrix
            control_matrix = np.mat([[forward_speed],
                                     [right_speed],
                                     [clockwise_val]])

            #Apply mecanum transformation
            output += self.MECHANUM_MATRIX * control_matrix

            #print(control_matrix)
            #print(output)

            wpilib.SmartDashboard.putNumber("left_front_wheel", output[0, 0])
            wpilib.SmartDashboard.putNumber("left_rear_wheel", output[1, 0])
            wpilib.SmartDashboard.putNumber("right_front_wheel", output[2, 0])
            wpilib.SmartDashboard.putNumber("right_rear_wheel", output[3, 0])
            wpilib.SmartDashboard.putNumber("Rotation PID", clockwise_val)

            #Send to motors.
#            self.motor_controllers[0].set(outputs[0, 0])
#            self.motor_controllers[1].set(outputs[0, 1])
#            self.motor_controllers[2].set(outputs[0, 2])
#            self.motor_controllers[3].set(outputs[0, 3])

            #Pause for a moment to let the rest of the code run
            yield from asyncio.sleep(.05)
        self.rot_pid.disable()
        #for controller in self.motor_controllers:
        #    controller.disableControl()
