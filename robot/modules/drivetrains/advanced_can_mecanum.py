import asyncio
import wpilib
import yeti
import math

from yeti.interfaces import gamemode, datastreams
from yeti.wpilib_extensions import Referee

class AdvancedCANMecanum(yeti.Module):
    """
    An advanced mecanum controller, taking input from the 'drivetrain_control' datastream
    and outputting to CAN Jaguars in closed-loop control mode.

    NOT FINISHED!!!!
    """

    ####################################
    # MOTOR CONTROLLER CONF

    #CAN IDS for the Drivetrain Jaguars in the following order:
    #Front Left
    #Rear Left
    #Front Right
    #Rear Right
    CAN_IDS = [13, 11, 12, 10]

    #Jagur PID Values
    JAG_P = 1.000
    JAG_I = 0.005
    JAG_D = 0.000

    ENCODER_TICKS_PER_ROTATION = 360

    #Multiply FPS by this value to get RPM with 6" wheels
    FPS_TO_RPM = 60/(.5 * math.pi)

    #######################################
    # GYRO CONF

    #Use Gyro
    USE_GYRO = True
    gyro_initialized = False

    GYRO_RATE_PID = False
    #Gyro rate PID Values
    GYRO_RATE_P = 0.500
    GYRO_RATE_I = 0.000
    GYRO_RATE_D = 0.000

    GYRO_RATE_PID_MAX_OUT = 1

    GYRO_POS_LOCK_PID = False
    #Gyro pos lock PID Values
    GYRO_POS_LOCK_P = 0.250
    GYRO_POS_LOCK_I = 0.010
    GYRO_POS_LOCK_D = 0.000


    def module_init(self):
        #Initialize the Referee for the module.
        self.referee = Referee(self)

        #Setup motor controllers
        self.motor_controllers = list()
        for motor_id in self.CAN_IDS:
            controller = wpilib.CANJaguar(motor_id)
            controller.setSpeedModeQuadEncoder(self.ENCODER_TICKS_PER_ROTATION, self.JAG_P, self.JAG_I, self.JAG_D)
            self.referee.watch(controller)
            self.motor_controllers.append(controller)

        #Start control datastream
        #Values are:
        # forward_fps -- desired forward speed in feet-per-second
        # right_fps -- desired right strafe speed in feet-per-second
        # clockwise_rps -- desired clockwise rotational speed in rotations-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")

        if self.USE_GYRO:
            self.gyro_init()

    gyro_rate_pid_out = 0
    gyro_pos_lock_pid_out = 0

    def gyro_init(self):
        self.logger.info("Starting Gyro Init")
        self.gyro = wpilib.Gyro(0)
        self.logger.info("Finished Gyro Init")
        self.referee.watch(self.gyro)

        if self.GYRO_RATE_PID:
            #Setup rotational rate PID controller
            def rate_pid_source():
                return self.gyro.getRate()/360

            def rate_pid_out(val):
                self.gyro_rate_pid_out += val
                if self.gyro_rate_pid_out > self.GYRO_RATE_PID_MAX_OUT:
                    self.gyro_rate_pid_out = self.GYRO_RATE_PID_MAX_OUT
                elif self.gyro_rate_pid_out < -self.GYRO_RATE_PID_MAX_OUT:
                    self.gyro_rate_pid_out = -self.GYRO_RATE_PID_MAX_OUT

            self.gyro_rate_pid_controller = wpilib.PIDController(self.GYRO_RATE_P, self.GYRO_RATE_I, self.GYRO_RATE_D, rate_pid_source, rate_pid_out)

        if self.GYRO_POS_LOCK_PID:
            #Setup rotational lock PID controller
            def pos_pid_source():
                return self.gyro.getAngle()

            def pos_pid_out(val):
                self.gyro_pos_lock_pid_out = val

            self.gyro_pos_lock_pid_controller = wpilib.PIDController(self.GYRO_POS_LOCK_P, self.GYRO_POS_LOCK_I, self.GYRO_POS_LOCK_D, pos_pid_source, pos_pid_out)

        #Set initialized flag
        self.gyro_initialized = True

    @yeti.autorun_coroutine
    @asyncio.coroutine
    def debug_data_loop(self):
        while True:
            i = 0
            for canjag in self.motor_controllers:
                wpilib.SmartDashboard.putNumber(str(i) + "-CanJagGet", canjag.get())
                wpilib.SmartDashboard.putNumber(str(i) + "-CanJagVolts", canjag.getOutputVoltage())
                wpilib.SmartDashboard.putNumber(str(i) + "-CanJagCurrent", canjag.getOutputCurrent())
                wpilib.SmartDashboard.putNumber(str(i) + "-CanJagSpeed", canjag.getSpeed())
                i += 1
            if self.gyro_initialized:
                wpilib.SmartDashboard.putNumber("GyroRate", self.gyro.getRate())
                wpilib.SmartDashboard.putNumber("GyroAngle", self.gyro.getAngle())

            yield from asyncio.sleep(.1)

    @gamemode.enabled_task
    @asyncio.coroutine
    def drive_loop(self):

        #Enable the gyro rate pid loop if it is initialized
        if self.gyro_initialized and self.GYRO_RATE_PID:
            self.gyro_rate_pid_controller.enable()

        #Enable CAN Jaguars
        for controller in self.motor_controllers:
            controller.enableControl()

        rotation_locked = False

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_speed = control_data.get("forward_fps", 0)
            right_speed = control_data.get("right_fps", 0)
            clockwise_speed = control_data.get("clockwise_rps", 0)

            #Rotation lock enable/disable
            if self.gyro_initialized and self.GYRO_POS_LOCK_PID:
                if clockwise_speed == 0:
                    if not rotation_locked:
                        rotation_locked = True
                        self.gyro_pos_lock_pid_controller.setSetpoint(self.gyro.getAngle())
                        self.gyro_pos_lock_pid_controller.enable()
                        if self.GYRO_RATE_PID:
                            self.gyro_rate_pid_controller.disable()
                else:
                    if rotation_locked:
                        rotation_locked = False
                        self.gyro_pos_lock_pid_controller.disable()
                        if self.GYRO_RATE_PID:
                            self.gyro_rate_pid_controller.enable()

            #Get rotation source
            if self.gyro_initialized and self.GYRO_POS_LOCK_PID and rotation_locked:
                clockwise_speed_out = self.gyro_pos_lock_pid_out
            elif self.gyro_initialized and self.GYRO_RATE_PID:
                self.gyro_rate_pid_controller.setSetpoint(clockwise_speed)
                clockwise_speed_out = self.gyro_rate_pid_out * 7
            else:
                clockwise_speed_out = clockwise_speed

            #Inverse kinematics to get mecanum values
            front_left_out = forward_speed + clockwise_speed_out + right_speed
            front_right_out = forward_speed - clockwise_speed_out - right_speed
            rear_left_out = forward_speed + clockwise_speed_out - right_speed
            rear_right_out = forward_speed - clockwise_speed_out + right_speed

            #convert from fps to rpm
            front_left_out *= self.FPS_TO_RPM
            front_right_out *= self.FPS_TO_RPM
            rear_left_out *= self.FPS_TO_RPM
            rear_right_out *= self.FPS_TO_RPM

            #Send to motors.
            self.motor_controllers[0].set(front_left_out)
            self.motor_controllers[1].set(rear_left_out)
            self.motor_controllers[2].set(-front_right_out)
            self.motor_controllers[3].set(-rear_right_out)

            #Save values to smartdashboard
            wpilib.SmartDashboard.putNumber("forward_fps", forward_speed)
            wpilib.SmartDashboard.putNumber("right_fps", right_speed)
            wpilib.SmartDashboard.putNumber("clockwise_speed", clockwise_speed)
            wpilib.SmartDashboard.putNumber("clockwise_speed_out", clockwise_speed_out)

            #Pause for a moment to let the rest of the code run
            yield from asyncio.sleep(.05)

        #Disable pid if used
        if self.gyro_initialized:
            if self.GYRO_RATE_PID:
                self.gyro_rate_pid_controller.disable()
            if self.GYRO_POS_LOCK_PID:
                self.gyro_pos_lock_pid_controller.disable()

        #Disable CAN Jaguars.
        for controller in self.motor_controllers:
            controller.disableControl()

    def module_deinit(self):
        if self.gyro_initialized:
            if self.GYRO_RATE_PID:
                self.gyro_rate_pid_controller.disable()
            if self.GYRO_POS_LOCK_PID:
                self.gyro_pos_lock_pid_controller.disable()
