import wpilib
import yeti

class ArcadeDrive(yeti.Module):
    """
    A bare-bones example of an arcade drive module.
    """

    def module_init(self):
        # Setup a joystick
        self.joystick = wpilib.Joystick(0)

        # Setup the robotdrive
        self.robotdrive = wpilib.RobotDrive(1, 2, 3, 4)

    def teleop_periodic(self):
        # Get the joystick values and drive the motors.
        self.robotdrive.arcadeDrive(-self.joystick.getY(), -self.joystick.getX())

    def module_deinit(self):
        self.robotdrive.free()
