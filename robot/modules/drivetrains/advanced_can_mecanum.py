import asyncio
import wpilib
import yeti
import math

from yeti.interfaces import gamemode, datastreams, remote_coroutines
from yeti.wpilib_extensions import Referee


#Helper funcs
def threshold_value(value, threshold):
    if abs(value) <= threshold:
        value = 0
    return value


def signing_square(value):
    if value < 0:
        return value ** 2
    else:
        return -(value ** 2)

def trapezoid_motion_profile(setpoint, position, speed, max_accel, max_speed):
    """

    """
    # Start by calculating down ramp at end, at maximum deceleration
    position_delta = setpoint - position

    # This is the speed that we would have to be travelling at this moment if we were
    # to continually decelerate -- reaching the setpoint exactly when speed is zero.
    #
    # This is absolute speed, still need to calculate direction.
    projected_speed = math.sqrt(2 * max_accel * abs(position_delta))

    # Now cap the previous value to the max_speed specified
    projected_speed = min(projected_speed, max_speed)

    # Calculate speed delta (Again forgetting direction of speed)
    speed_delta = projected_speed - abs(speed)




    # Now limit the speed to



class AdvancedCANMecanum(yeti.Module):
    """
    An advanced mecanum controller, taking input from the 'drivetrain_control' datastream
    and outputting to CAN Jaguars in closed-loop control mode.

    """


    ####################################
    # JOYSTICK CONTROLLER CONF

    #Maximum values for joystick loop to output

    MAX_Y_INPUT_FPS = 10
    MAX_X_INPUT_FPS = 10
    MAX_ROT_INPUT_DPS = 360

    SQUARE_INPUTS = True

    ####################################
    # MOTOR CONTROLLER CONF

    #CAN IDS for the Drivetrain Jaguars in the following order:
    #Front Left
    #Rear Left
    #Front Right
    #Rear Right
    CAN_IDS = [12, 14, 11, 13]

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
    USE_GYRO = False
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

        self.joystick = wpilib.Joystick(0)
        self.referee.watch(self.joystick)

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
        # clockwise_dps -- desired clockwise rotational speed in degrees-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")

        self.auto_drive_setpoint_datastream = datastreams.get_datastream("auto_drive_setpoint")
        self.auto_drive_input_datastream = datastreams.get_datastream("auto_drive_input")

        self.auto_drive_config = [{"speed": 10, "acceleration": 10, "tolerance": .1, "use_tracker": True},
                                  {"speed": 10, "acceleration": 10, "tolerance": .1, "use_tracker": True}]

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
                return self.gyro.getRate()

            def rate_pid_out(val):

                self.gyro_rate_pid_out = val

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

            yield from asyncio.sleep(.2)

    #@yeti.autorun_coroutine
    @asyncio.coroutine
    def tracking_loop(self):
        last_position = [0, 0]
        last_gyro_angle = 0
        if self.gyro_initialized:
            self.gyro.reset()
        last_wheel_positions = [c.getPosition() for c in self.motor_controllers]
        last_cycle_timestamp = wpilib.Timer.getFPGATimestamp()
        while True:
            yield from asyncio.sleep(.05)
            current_cycle_timestamp = wpilib.Timer.getFPGATimestamp()
            delta_time = current_cycle_timestamp - last_cycle_timestamp
            last_cycle_timestamp = current_cycle_timestamp

            # Track wheel movement and get distances and average speeds
            current_wheel_positions = []
            current_wheel_positions[0] = self.motor_controllers[0].getPosition()
            current_wheel_positions[1] = self.motor_controllers[1].getPosition()
            current_wheel_positions[2] = -self.motor_controllers[2].getPosition()
            current_wheel_positions[3] = -self.motor_controllers[3].getPosition()

            delta_wheel_positions = [c - l for c, l in zip(current_wheel_positions, last_wheel_positions)]
            last_wheel_positions = current_wheel_positions
            average_wheel_speeds = [d * delta_time for d in delta_wheel_positions]

            # Get gyro state and get average angle
            current_gyro_angle = self.gyro.getAngle()
            average_gyro_angle = last_gyro_angle + current_gyro_angle / 2
            last_gyro_angle = current_gyro_angle

            # Calculate cartesian velocity of translation
            trans_y_speed = average_wheel_speeds[0]/4 + average_wheel_speeds[1]/4 + average_wheel_speeds[2]/4 + average_wheel_speeds[3]/4
            trans_x_speed = average_wheel_speeds[0]/4 - average_wheel_speeds[1]/4 - average_wheel_speeds[2]/4 + average_wheel_speeds[3]/4

            # Convert to polar coordinates
            trans_angle = math.atan2(trans_y_speed, trans_x_speed)
            trans_speed = math.sqrt(trans_x_speed ** 2 + trans_y_speed ** 2)

            # Convert to world coordinates
            world_trans_angle = trans_angle + average_gyro_angle

            # Convert speed to distance
            world_trans_distance = trans_speed * delta_time

            # Convert back to cartesian coordinates
            world_trans_y_dist = math.sin(world_trans_angle) * world_trans_distance
            world_trans_x_dist = math.cos(world_trans_angle) * world_trans_distance

            # Add to last values
            last_position[0] += world_trans_x_dist
            last_position[1] += world_trans_y_dist

            # Set input datastream if configured to do so
            input_stream_updates = {}
            if self.auto_drive_config[0]["use_tracker"]:
                input_stream_updates["x_pos"] = last_position[0]
            if self.auto_drive_config[1]["use_tracker"]:
                input_stream_updates["y_pos"] = last_position[1]
            if len(input_stream_updates) != 0:
                self.auto_drive_input_datastream.push(input_stream_updates)

    @asyncio.coroutine
    @remote_coroutines.public_coroutine
    def auto_drive_x_config(self, config):
        self.auto_drive_config[0].update(config)

    @asyncio.coroutine
    @remote_coroutines.public_coroutine
    def auto_drive_y_config(self, config):
        self.auto_drive_config[1].update(config)

    auto_drive_enabled = False

    @asyncio.coroutine
    @remote_coroutines.public_coroutine
    def enable_auto_drive(self):
        self.auto_drive_enabled = True
        self.start_coroutine(self.auto_drive_loop())

    @asyncio.coroutine
    @remote_coroutines.public_coroutine
    def disable_auto_drive(self):
        self.auto_drive_enabled = False

    @asyncio.coroutine
    def auto_drive_loop(self):
        while self.auto_drive_enabled:
            # This is currently a square algorithm, it should be upgraded to trapezoidal

            # Get inputs and setpoints
            input_data = self.auto_drive_input_datastream.get()
            setpoint_data = self.auto_drive_setpoint_datastream.get()
            current_input = [input_data.get("x_pos"), input_data.get("y_pos")]
            current_setpoint = [setpoint_data.get("x_pos"), setpoint_data.get("y_pos")]




            assert False

            asyncio.sleep(.05)



    @gamemode.teleop_task
    @asyncio.coroutine
    def joystick_loop(self):
        while gamemode.is_teleop():
            forward_percentage = self.joystick.getY()
            right_percentage = -self.joystick.getX()
            clockwise_percentage = -self.joystick.getZ()

            #Threshold values
            forward_percentage = threshold_value(forward_percentage, .10)
            right_percentage = threshold_value(right_percentage, .10)
            clockwise_percentage = threshold_value(clockwise_percentage, .15)

            if self.SQUARE_INPUTS:
                forward_percentage = signing_square(forward_percentage)
                right_percentage = signing_square(right_percentage)
                clockwise_percentage = signing_square(clockwise_percentage)

            #Scale to real-world measurments
            forward_fps = forward_percentage * self.MAX_Y_INPUT_FPS
            right_fps = right_percentage * self.MAX_X_INPUT_FPS
            clockwise_dps = clockwise_percentage * self.MAX_ROT_INPUT_DPS

            #Send values to drive loop
            self.control_datastream.push({"forward_fps": forward_fps, "right_fps": right_fps, "clockwise_dps": clockwise_dps})

            yield from asyncio.sleep(.05)

    @gamemode.enabled_task
    @asyncio.coroutine
    def drive_loop(self):

        #Clear datastream
        self.control_datastream.push({"forward_fps": 0, "right_fps": 0, "clockwise_dps": 0})

        #Enable the gyro rate pid loop if it is initialized
        if self.gyro_initialized:
            if self.GYRO_RATE_PID:
                self.gyro_rate_pid_controller.enable()
            if self.GYRO_POS_LOCK_PID:
                self.gyro_pos_lock_pid_controller.disable()

        #Enable CAN Jaguars
        for controller in self.motor_controllers:
            controller.enableControl()

        rotation_locked = False

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_speed = control_data.get("forward_fps", 0)
            right_speed = control_data.get("right_fps", 0)
            clockwise_speed = control_data.get("clockwise_dps", 0)

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
                clockwise_speed_out = clockwise_speed / 360

            #Inverse kinematics to get mecanum values
            front_left_out = forward_speed + clockwise_speed_out + right_speed
            rear_left_out = forward_speed + clockwise_speed_out - right_speed
            front_right_out = forward_speed - clockwise_speed_out - right_speed
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
