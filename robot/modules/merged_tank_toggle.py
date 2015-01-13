__author__ = 'Tim'
import asyncio
import wpilib
import yeti

from yeti.interfaces import gamemode
from yeti.wpilib_extensions import Referee

class tank_toggleDrive(yeti.Module):
    """
    A 1-2 Joystick Mecanum drivetrain module, holding button 10 to switch between tank and toggle drive, holding button 9 to switch between spin and strafe.
    """

    USE_CAN = True
    driveswitch = True
    drivemode = 0

    def module_init(self):
        #Initialize the Referee for the module.
        self.referee = Referee(self)

        #Setup a joystick
        self.joystick_left = wpilib.Joystick(0)
        self.referee.watch(self.joystick_left)
        self.joystick_right = wpilib.Joystick(1)
        self.referee.watch(self.joystick_right)

        if self.USE_CAN:
            motor_controller_class = wpilib.CANJaguar
            self.right_front_cim = motor_controller_class(10)
            self.referee.watch(self.right_front_cim)

            self.left_front_cim = motor_controller_class(11)
            self.referee.watch(self.left_front_cim)

            self.right_rear_cim = motor_controller_class(12)
            self.referee.watch(self.right_rear_cim)

            self.left_rear_cim = motor_controller_class(13)
            self.referee.watch(self.left_rear_cim)
        else:
            motor_controller_class = wpilib.Jaguar
            self.right_front_cim = motor_controller_class(0)
            self.referee.watch(self.right_front_cim)

            self.left_front_cim = motor_controller_class(1)
            self.referee.watch(self.left_front_cim)

            self.right_rear_cim = motor_controller_class(2)
            self.referee.watch(self.right_rear_cim)

            self.left_rear_cim = motor_controller_class(3)
            self.referee.watch(self.left_rear_cim)

        #Setup the robotdrive
        self.robotdrive = wpilib.RobotDrive(self.left_front_cim, self.left_rear_cim, self.right_front_cim, self.right_rear_cim)
        self.referee.watch(self.robotdrive)


    @gamemode.teleop_task
    @asyncio.coroutine
    def teleop_loop(self):
        ldsv = True
        #Loop until end of teleop mode.
        while gamemode.is_teleop():
            if self.driveswitch:
                if not ldsv and self.joystick_left.getRawButton(9):
                    self.drivemode += 1
                    if self.drivemode > 1:
                        self.drivemode = 0


            if self.drivemode == 0:
                ly = self.joystick_left.getY()
                lx = self.joystick_left.getX()
                ry = self.joystick_right.getY()
                rx = self.joystick_right.getX()

                y_out = (ly + ry)/2
                x_out = (lx + rx)/2
                turn_out = (ly - ry)/2

                self.robotdrive.mecanumDrive_Cartesian(x_out, y_out, turn_out, 0)

            elif self.drivemode == 1:
                strafemode = self.joystick_left.getRawButton(8)

                if strafemode:
                    self.robotdrive.mecanumDrive_Cartesian(-self.joystick_left.getX(), -self.joystick_left.getY(), 0, 0)
                else:
                    self.robotdrive.mecanumDrive_Cartesian(0, -self.joystick_left.getY(), -self.joystick_left.getX(), 0)

                #Pause for a moment to let the rest of the code run.
            ldsv = self.joystick_left.getRawButton(9)
            yield from asyncio.sleep(.05)
