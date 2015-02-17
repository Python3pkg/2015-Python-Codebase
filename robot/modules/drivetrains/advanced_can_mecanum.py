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

# This code is directly ported from team 254's 2011 ContinuousAccelFilter


def continuous_accel_filter(distance_to_target, current_vel, goal_vel, max_accel, max_vel, delta_time):

    output_velocity = current_vel

    start_accel_time, start_accel, const_time, end_accel_time, end_accel = calculate_motion_profile(distance_to_target, current_vel, goal_vel, max_accel, max_vel)

    time_left = delta_time
    output_velocity += start_accel * min(start_accel_time, time_left)
    if start_accel_time < time_left:
        time_left -= start_accel_time
        if const_time < time_left:
            time_left -= const_time
            output_velocity += start_accel * min(end_accel_time, time_left)
            if end_accel_time < time_left:
                time_left -= end_accel_time
                output_velocity += end_accel * time_left

    return output_velocity


def calculate_motion_profile(distance_left, current_vel, goal_vel, max_accel, max_vel):
    """
    Does magical stuff, still figuring it out myself.

    :param distance_left: The distance left to traverse
    :param current_vel: The current velocity of the robot
    :param goal_vel: The goal velocity of the robot
    :param max_accel: The maximum desired acceleration of the robot
    :param max_vel: The maximum desired velocity of the robot
    """

    # Return variables
    start_accel_time = 0
    start_accel_rate = 0
    const_time = 0
    end_accel_time = 0
    end_accel_rate = 0

    if distance_left > 0:
        start_accel_rate = max_accel

    elif distance_left == 0:
        # We are at the setpoint, so zero everything and return
        start_accel_time = 0
        start_accel_rate = 0
        const_time = 0
        end_accel_time = 0
        end_accel_rate = 0
        return start_accel_time, \
            start_accel_rate, \
            const_time, \
            end_accel_time, \
            end_accel_rate

    else:
        start_accel_time, \
            start_accel_rate, \
            const_time, \
            end_accel_time, \
            end_accel_rate = calculate_motion_profile(-distance_left, -current_vel, -goal_vel, max_accel, max_vel)
        start_accel_rate *= -1
        end_accel_rate *= -1
        return start_accel_time, \
            start_accel_rate, \
            const_time, \
            end_accel_time, \
            end_accel_rate

    # The velocity I would be travelling if I were to accelerate at max acceleration until there were no distance left
    max_accel_velocity = distance_left * 2 * abs(start_accel_rate) + current_vel ** 2

    if max_accel_velocity > 0:
        max_accel_velocity = math.sqrt(max_accel_velocity)
    else:
        max_accel_velocity = -math.sqrt(-max_accel_velocity)

    if max_accel_velocity > goal_vel:
        end_accel_rate = -max_accel
    else:
        end_accel_rate = max_accel

    # This formula is shrouded in magic and mystery. I cannot seem to understand how it does it, but it does.
    # This calculates the top speed at which we can go given the distance and acceleration constraints.
    top_vel = math.sqrt((distance_left + (current_vel ** 2) / (2.0 * start_accel_rate) + (goal_vel ** 2) / (2.0 * end_accel_rate)) / (-1.0 / (2.0 * end_accel_rate) + 1.0 / (2.0 * start_accel_rate)));

    # If top_vel is too fast, we now know how long we get to accelerate for and how long to go at constant velocity
    if top_vel > max_vel:
        start_accel_time = (max_vel - current_vel) / max_accel
        const_time = (distance_left + (goal_vel ** 2 - max_vel ** 2) / (2*max_accel)) / max_vel
    else:
        start_accel_time = (top_vel - current_vel) / start_accel_rate

    return start_accel_time, \
        start_accel_rate, \
        const_time, \
        end_accel_time, \
        end_accel_rate

# End ported code


class AdvancedCANMecanum(yeti.Module):
    """
    An advanced mecanum controller, taking input from the 'drivetrain_control' datastream
    and outputting to CAN Jaguars in closed-loop control mode.

    """

    SIMULATE_IO = False

    ####################################
    # JOYSTICK CONTROLLER CONF

    # Maximum values for joystick loop to output

    MAX_Y_INPUT_FPS = 10
    MAX_X_INPUT_FPS = 10
    MAX_ROT_INPUT_DPS = 360

    SQUARE_INPUTS = True

    ####################################
    # MOTOR CONTROLLER CONF

    # CAN IDS for the Drivetrain Jaguars in the following order:
    # Front Left
    # Rear Left
    # Front Right
    # Rear Right
    CAN_IDS = [12, 14, 11, 13]

    # Jagur PID Values
    JAG_P = 1.000
    JAG_I = 0.005
    JAG_D = 0.000

    ENCODER_TICKS_PER_ROTATION = 360

    # Multiply FPS by this value to get RPM with 6" wheels
    FPS_TO_RPM = 60/(.5 * math.pi)

    #######################################
    # GYRO CONF

    # Use Gyro
    USE_GYRO = False
    gyro_initialized = False

    GYRO_RATE_PID = False
    # Gyro rate PID Values

    GYRO_RATE_P = 0.100
    GYRO_RATE_I = 0.000
    GYRO_RATE_D = 0.000

    GYRO_POS_LOCK_PID = False
    # Gyro pos lock PID Values
    GYRO_POS_LOCK_P = 1.000
    GYRO_POS_LOCK_I = 0.100
    GYRO_POS_LOCK_D = 0.000


    def module_init(self):
        # Initialize the Referee for the module.
        self.referee = Referee(self)

        self.joystick = wpilib.Joystick(0)
        self.referee.watch(self.joystick)

        if self.SIMULATE_IO:
            self.motor_controllers = list()
            for motor_id in range(4):
                controller = wpilib.Talon(motor_id)
                self.referee.watch(controller)
                self.motor_controllers.append(controller)
        else:
            # Setup motor controllers
            self.motor_controllers = list()
            for motor_id in self.CAN_IDS:
                controller = wpilib.CANJaguar(motor_id)
                controller.setSpeedModeQuadEncoder(self.ENCODER_TICKS_PER_ROTATION, self.JAG_P, self.JAG_I, self.JAG_D)
                self.referee.watch(controller)
                self.motor_controllers.append(controller)

        # Start control datastream
        # Values are:
        #  forward_fps -- desired forward speed in feet-per-second
        #  right_fps -- desired right strafe speed in feet-per-second
        #  clockwise_dps -- desired clockwise rotational speed in degrees-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")

        self.auto_drive_setpoint_datastream = datastreams.get_datastream("auto_drive_setpoint")
        self.auto_drive_input_datastream = datastreams.get_datastream("auto_drive_input")

        self.auto_drive_config = {"x_speed": 10, "x_acceleration": 15, "x_tolerance": .1, "x_use_tracker": True,
                                  "y_speed": 10, "y_acceleration": 15, "y_tolerance": .1, "y_use_tracker": True}

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

            self.gyro_rate_pid_controller = wpilib.PIDController(self.GYRO_RATE_P, self.GYRO_RATE_I, self.GYRO_RATE_D, 1, rate_pid_source, rate_pid_out)
            self.gyro_rate_pid_controller.setOutputRange(-360, 360)

        if self.GYRO_POS_LOCK_PID:
            #Setup rotational lock PID controller
            def pos_pid_source():
                return self.gyro.getAngle()

            def pos_pid_out(val):
                self.gyro_pos_lock_pid_out = val

            self.gyro_pos_lock_pid_controller = wpilib.PIDController(self.GYRO_POS_LOCK_P, self.GYRO_POS_LOCK_I, self.GYRO_POS_LOCK_D, pos_pid_source, pos_pid_out)
            self.gyro_pos_lock_pid_controller.setOutputRange(-360, 360)

        #Set initialized flag
        self.gyro_initialized = True

    @yeti.autorun_coroutine
    @asyncio.coroutine
    def debug_data_loop(self):
        while True:
            i = 0
            for canjag in self.motor_controllers:
                wpilib.SmartDashboard.putNumber(str(i) + "-CanJagGet", canjag.get())
                if not self.SIMULATE_IO:
                    wpilib.SmartDashboard.putNumber(str(i) + "-CanJagVolts", canjag.getOutputVoltage())
                    wpilib.SmartDashboard.putNumber(str(i) + "-CanJagCurrent", canjag.getOutputCurrent())
                    wpilib.SmartDashboard.putNumber(str(i) + "-CanJagSpeed", canjag.getSpeed())
                i += 1
            if self.gyro_initialized:
                wpilib.SmartDashboard.putNumber("GyroRate", self.gyro.getRate())
                wpilib.SmartDashboard.putNumber("GyroAngle", self.gyro.getAngle())

            yield from asyncio.sleep(.2)

    tracking_reset = True

    @remote_coroutines.public_coroutine
    def reset_tracking(self):
        self.tracking_reset = False


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
            if self.tracking_reset:
                last_position = [0, 0]
                self.gyro.reset()
                self.tracking_reset = False
                yield from asyncio.sleep(.05)
            current_cycle_timestamp = wpilib.Timer.getFPGATimestamp()
            delta_time = current_cycle_timestamp - last_cycle_timestamp
            last_cycle_timestamp = current_cycle_timestamp

            # Track wheel movement and get distances and average speeds
            if self.SIMULATE_IO:
                average_wheel_speeds = [0, 0, 0, 0]
                average_wheel_speeds[0] = self.motor_controllers[0].get() * 14
                average_wheel_speeds[1] = self.motor_controllers[1].get() * 14
                average_wheel_speeds[2] = -self.motor_controllers[2].get() * 14
                average_wheel_speeds[3] = -self.motor_controllers[3].get() * 14

            else:
                current_wheel_positions = [0, 0, 0, 0]
                current_wheel_positions[0] = self.motor_controllers[0].getPosition()
                current_wheel_positions[1] = self.motor_controllers[1].getPosition()
                current_wheel_positions[2] = -self.motor_controllers[2].getPosition()
                current_wheel_positions[3] = -self.motor_controllers[3].getPosition()

                delta_wheel_positions = [c - l for c, l in zip(current_wheel_positions, last_wheel_positions)]
                last_wheel_positions = current_wheel_positions[:]
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

            wpilib.SmartDashboard.putNumber("tracked_x_pos", last_position[0])
            wpilib.SmartDashboard.putNumber("tracked_y_pos", last_position[1])

            # Set input datastream if configured to do so
            input_stream_updates = {}
            if self.auto_drive_config["x_use_tracker"]:
                input_stream_updates["x_pos"] = last_position[0]
            if self.auto_drive_config["y_use_tracker"]:
                input_stream_updates["y_pos"] = last_position[1]
            if len(input_stream_updates) != 0:
                self.auto_drive_input_datastream.push(input_stream_updates)

    auto_drive_enabled = False

    @remote_coroutines.public_coroutine
    def auto_drive_x_at_setpoint(self):
        x_pos = self.auto_drive_input_datastream.get().get("x_pos", 0)
        x_setpoint = self.auto_drive_setpoint_datastream.get().get("x_pos", 0)
        x_tolerance = self.auto_drive_config["x_tolerance"]
        return abs(x_pos - x_setpoint) < x_tolerance

    @remote_coroutines.public_coroutine
    def auto_drive_y_at_setpoint(self):
        y_pos = self.auto_drive_input_datastream.get().get("y_pos", 0)
        y_setpoint = self.auto_drive_setpoint_datastream.get().get("y_pos", 0)
        y_tolerance = self.auto_drive_config["y_tolerance"]
        return abs(y_pos - y_setpoint) < y_tolerance

    @remote_coroutines.public_coroutine
    @asyncio.coroutine
    def auto_drive_wait_for_x(self):
        while not self.auto_drive_x_at_setpoint():
            yield from asyncio.sleep(.1)

    @remote_coroutines.public_coroutine
    @asyncio.coroutine
    def auto_drive_wait_for_y(self):
        while not self.auto_drive_y_at_setpoint():
            yield from asyncio.sleep(.1)

    @asyncio.coroutine
    @remote_coroutines.public_coroutine
    def auto_drive_enable(self):
        if not self.auto_drive_enabled:
            self.auto_drive_enabled = True
            self.start_coroutine(self.auto_drive_loop())

    @asyncio.coroutine
    @remote_coroutines.public_coroutine
    def auto_drive_disable(self):
        self.auto_drive_enabled = False

    @asyncio.coroutine
    def auto_drive_loop(self):
        last_input_x = 0
        last_input_y = 0
        last_cycle_time = wpilib.Timer.getFPGATimestamp() - .05

        while self.auto_drive_enabled:

            current_cycle_time = wpilib.Timer.getFPGATimestamp()
            delta_time = current_cycle_time - last_cycle_time
            last_cycle_time = current_cycle_time

            # Get inputs and setpoints
            input_data = self.auto_drive_input_datastream.get()
            setpoint_data = self.auto_drive_setpoint_datastream.get()

            max_speed_x = self.auto_drive_config["x_speed"]
            max_speed_y = self.auto_drive_config["y_speed"]

            max_accel_x = self.auto_drive_config["x_acceleration"]
            max_accel_y = self.auto_drive_config["y_acceleration"]

            x_tolerance = self.auto_drive_config["x_tolerance"]
            y_tolerance = self.auto_drive_config["y_tolerance"]

            current_input_x = input_data.get("x_pos")
            current_input_y = input_data.get("y_pos")

            current_setpoint_x = setpoint_data.get("x_pos", 0)
            current_setpoint_y = setpoint_data.get("y_pos", 0)

            x_delta = current_setpoint_x - current_input_x
            y_delta = current_setpoint_y - current_input_y

            x_avg_speed = (current_input_x - last_input_x) / delta_time
            y_avg_speed = (current_input_y - last_input_y) / delta_time

            if abs(x_delta) > x_tolerance:
                x_speed_out = continuous_accel_filter(x_delta, x_avg_speed, 0, max_accel_x, max_speed_x, delta_time)
            else:
                x_speed_out = 0

            if abs(y_delta) > y_tolerance:
                y_speed_out = continuous_accel_filter(y_delta, y_avg_speed, 0, max_accel_y, max_speed_y, delta_time)
            else:
                y_speed_out = 0

            last_input_x = current_input_x
            last_input_y = current_input_y

            # Send values to drive loop
            self.control_datastream.push({"forward_fps": y_speed_out, "right_fps": x_speed_out, "clockwise_dps": 0})

            yield from asyncio.sleep(.1)

        self.control_datastream.push({"forward_fps": 0, "right_fps": 0, "clockwise_dps": 0})




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
            clockwise_percentage = threshold_value(clockwise_percentage, .25)

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
        if not self.SIMULATE_IO:
            for controller in self.motor_controllers:
                controller.enableControl()

        rotation_locked = False

        while gamemode.is_enabled():
            #Get control inputs
            control_data = self.control_datastream.get()
            forward_speed = control_data.get("forward_fps", 0)
            right_speed = control_data.get("right_fps", 0)
            clockwise_speed_in = control_data.get("clockwise_dps", 0)

            #Rotation lock enable/disable
            if self.gyro_initialized and self.GYRO_POS_LOCK_PID:
                if clockwise_speed_in == 0:
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
                self.gyro_rate_pid_controller.setSetpoint(clockwise_speed_in)
                clockwise_speed_out = self.gyro_rate_pid_out
            else:
                clockwise_speed_out = clockwise_speed_in

            #Inverse kinematics to get mecanum values

            # This is abs(x_dist) + abs(y_dist), where x_dist and y_dist are the x and y distances of the
            # mecanum wheels from the center of mass of the robot. x_dist is 12" and y_dist is 15", so
            # the result is 27", which equals 2.25'. This is the multiplier for a clockwise speed measured
            # in radians, so we multiply it by pi/180 to get the multiplier for a clockwise speed measured
            # in degrees.
            rot_scale = 0.03927

            front_left_out = forward_speed + right_speed + (clockwise_speed_out * rot_scale)
            rear_left_out = forward_speed - right_speed + (clockwise_speed_out * rot_scale)
            front_right_out = forward_speed - right_speed - (clockwise_speed_out * rot_scale)
            rear_right_out = forward_speed + right_speed - (clockwise_speed_out * rot_scale)


            #Send to motors.
            if self.SIMULATE_IO:
                self.motor_controllers[0].set(front_left_out/14)
                self.motor_controllers[1].set(rear_left_out/14)
                self.motor_controllers[2].set(-front_right_out/14)
                self.motor_controllers[3].set(-rear_right_out/14)
            else:
                #convert from fps to rpm
                front_left_out *= self.FPS_TO_RPM
                front_right_out *= self.FPS_TO_RPM
                rear_left_out *= self.FPS_TO_RPM
                rear_right_out *= self.FPS_TO_RPM

                self.motor_controllers[0].set(front_left_out)
                self.motor_controllers[1].set(rear_left_out)
                self.motor_controllers[2].set(-front_right_out)
                self.motor_controllers[3].set(-rear_right_out)

            #Save values to smartdashboard
            wpilib.SmartDashboard.putNumber("forward_fps", forward_speed)
            wpilib.SmartDashboard.putNumber("right_fps", right_speed)
            wpilib.SmartDashboard.putNumber("clockwise_speed", clockwise_speed_in)
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
        if not self.SIMULATE_IO:
            for controller in self.motor_controllers:
                controller.disableControl()

    def module_deinit(self):
        if self.gyro_initialized:
            if self.GYRO_RATE_PID:
                self.gyro_rate_pid_controller.disable()
            if self.GYRO_POS_LOCK_PID:
                self.gyro_pos_lock_pid_controller.disable()
