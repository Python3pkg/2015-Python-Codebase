import yeti
import math
import wpilib

class AdvancedCanMecanum2(yeti.Module):

    ####################################
    # JOYSTICK CONTROLLER CONF

    # Maximum values for joystick loop to output

    FAST_JOYSTICK_Y_FPS = 10
    FAST_JOYSTICK_X_FPS = 10
    FAST_JOYSTICK_R_DPS = 360

    SLOW_JOYSTICK_Y_FPS = 3
    SLOW_JOYSTICK_X_FPS = 3
    SLOW_JOYSTICK_R_DPS = 90

    SQUARE_INPUTS = True

    ####################################
    # MOTOR CONTROLLER CONF

    # CAN IDS for the Drivetrain Jaguars in the following order:
    # Front Left
    # Rear Left
    # Front Right
    # Rear Right
    CAN_IDS = [12, 14, 15, 13]

    # Jagur PID Values
    JAG_P = 1.000
    JAG_I = 0.005
    JAG_D = 0.000

    ENCODER_TICKS_PER_ROTATION = 360

    # Multiply FPS by this value to get RPM with 6" wheels
    ROT_TO_FEET = .5 * math.pi
    FPS_TO_RPM = 60/ROT_TO_FEET

    #######################################
    # GYRO CONF

    # Use Gyro
    USE_GYRO = True

    ########################
    # DEBUG OPTIONS

    # Enables submitting debug metrics to NetworkTables
    DEBUG_NT_OUT = True

    ########################
    # PRE-CALCULATED VALUES

    # This is abs(x_dist) + abs(y_dist), where x_dist and y_dist are the x and y distances of the
    # mecanum wheels from the center of mass of the robot. x_dist is 12" and y_dist is 15", so
    # the result is 27", which equals 2.25'. This is the multiplier for a clockwise speed measured
    # in radians, so we multiply it by pi/180 to get the multiplier for a clockwise speed measured
    # in degrees.
    mecanum_kinematic_k = 0.03927


    def module_init(self):
        self.joystick = wpilib.Joystick(0)

        self.pipeline = self.engine.get_module("pipeline")

        # Setup motor controllers
        self.motor_controllers = list()
        for motor_id in self.CAN_IDS:
            controller = wpilib.CANJaguar(motor_id)
            self.motor_controllers.append(controller)
        self.set_speed_mode()

        self.reset_auto_config()

        if self.USE_GYRO:
            self.logger.info("Starting Gyro Init")
            self.gyro = wpilib.Gyro(0)
            self.logger.info("Finished Gyro Init")

    def set_pid(self, p, i, d):
        self.JAG_P = p
        self.JAG_I = i
        self.JAG_D = d

    reset_jaguar_input_flag = False
    def set_speed_mode(self):
        if self.motor_controllers[0].getControlMode() != wpilib.CANJaguar.ControlMode.Speed:
            self.logger.info("Set jaguars to speed mode.")
            for controller in self.motor_controllers:
                controller.disableControl()
                controller.setSpeedModeQuadEncoder(self.ENCODER_TICKS_PER_ROTATION, self.JAG_P, self.JAG_I, self.JAG_D)
                controller.enableControl()
                self.reset_jaguar_input_flag = True

    def set_percent_mode(self):
        if self.motor_controllers[0].getControlMode() != wpilib.CANJaguar.ControlMode.PercentVbus:
            self.logger.info("Set jaguars to percent mode.")
            for controller in self.motor_controllers:
                controller.disableControl()
                controller.setPercentModeQuadEncoder(self.ENCODER_TICKS_PER_ROTATION)
                controller.enableControl()
                self.reset_jaguar_input_flag = True