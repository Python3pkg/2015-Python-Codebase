import asyncio
import wpilib
import yeti
import math

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import public_object
from yeti.wpilib_extensions import Referee

class SimulatedCANJaguar():

    MAX_RPM_OUTPUT = 500

    def __init__(self, CAN_ID):
        self.talon = wpilib.Talon(CAN_ID - 10)
        self.position = 0
        self.last_update = wpilib.Timer.getFPGATimestamp()
        self.speed = 0
        self.mode = wpilib.CANJaguar.ControlMode.PercentVbus
        self.enabled = False

    def _reset_physics(self):
        self.position = 0

    def setSpeedModeQuadEncoder(self, codesPerRev, p, i, d):
        self.mode = wpilib.CANJaguar.ControlMode.Speed
        self.setPID(p, i, d)
        self._reset_physics()

    def setPercentModeQuadEncoder(self, codesPerRev):
        self.mode = wpilib.CANJaguar.ControlMode.PercentVbus
        self._reset_physics()

    def _update_physics(self):
        current_time = wpilib.Timer.getFPGATimestamp()
        delta_time = current_time - self.last_update
        self.position += self.speed * (delta_time/60)
        self.last_update = current_time

    def set(self, value):
        self._update_physics()
        if self.enabled:
            if self.mode == wpilib.CANJaguar.ControlMode.Speed:
                self.speed = value
                self.talon.set(value / self.MAX_RPM_OUTPUT)
            elif self.mode == wpilib.CANJaguar.ControlMode.PercentVbus:
                self.speed = value * self.MAX_RPM_OUTPUT
                self.talon.set(value)

    def get(self):
        return self.speed

    def setPID(self, p, i, d):
        print("Set P: {}, I: {}, D{}".format(p, i, d))

    def getControlMode(self):
        return self.mode

    def getSpeed(self):
        return self.speed

    def getPosition(self):
        self._update_physics()
        return self.position

    def enableControl(self):
        self._update_physics()
        self.enabled = True

    def disableControl(self):
        self._update_physics()
        self.enabled = False
        self.speed = 0

    def getOutputCurrent(self):
        return 0

    def getOutputVoltage(self):
        return 0

    def setNeuteralMode(self, mode):
        pass

    def free(self):
        self.talon.free()


# Helper funcs
def threshold_value(value, threshold):
    if abs(value) <= threshold:
        value = 0
    return value


def signing_square(value):
    if value < 0:
        return -(value ** 2)
    else:
        return value ** 2


def cartesian_to_polar(x, y):
    angle = math.atan2(y, x)*180/math.pi
    magnitude = math.sqrt(x ** 2 + y ** 2)
    return angle, magnitude


def polar_to_cartesian(angle, magnitude):
    angle *= math.pi/180
    y = math.sin(angle) * magnitude
    x = math.cos(angle) * magnitude
    return x, y


def rotate_vector(x, y, angle):
    vector_angle, vector_magnitude = cartesian_to_polar(x, y)
    return polar_to_cartesian(angle + vector_angle, vector_magnitude)


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

    USE_SIMULATED_JAGUAR = True

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
        # Initialize the Referee for the module.
        self.referee = Referee(self)

        self.joystick = wpilib.Joystick(0)
        self.referee.watch(self.joystick)

        # Setup motor controllers
        self.motor_controllers = list()
        for motor_id in self.CAN_IDS:
            if self.USE_SIMULATED_JAGUAR:
                controller = SimulatedCANJaguar(motor_id)
            else:
                controller = wpilib.CANJaguar(motor_id)
            self.referee.watch(controller)
            self.motor_controllers.append(controller)
        self.set_speed_mode()

        # Start control datastream
        # Values are:
        #  forward_fps -- desired forward speed in feet-per-second
        #  right_fps -- desired right strafe speed in feet-per-second
        #  ctrclockwise_dps -- desired counterclockwise rotational speed in degrees-per-second
        self.control_datastream = datastreams.get_datastream("drivetrain_control")
        self.sensor_input_datastream = datastreams.get_datastream("drivetrain_sensor_input")

        self.external_sensor_input_datastream = datastreams.get_datastream("drivetrain_external_sensor_input")

        self.autodrive_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.autodrive_config_datastream = datastreams.get_datastream("drivetrain_auto_config")
        self.reset_auto_config()

        if self.USE_GYRO:
            self.logger.info("Starting Gyro Init")
            self.gyro = wpilib.Gyro(0)
            self.logger.info("Finished Gyro Init")
            self.referee.watch(self.gyro)

    @public_object(prefix="drivetrain")
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

    reset_sensor_input_flag = False
    reset_x_pos = 0
    reset_y_pos = 0
    reset_r_pos = 0
    @public_object(prefix="drivetrain")
    def reset_sensor_input(self, x_pos=0, y_pos=0, r_pos=0):
        self.reset_x_pos = x_pos
        self.reset_y_pos = y_pos
        self.reset_r_pos = r_pos
        self.sensor_input_datastream.push({"x_pos": 0, "x_speed": 0,
                                           "y_pos": 0, "y_speed": 0,
                                           "r_pos": 0, "r_speed": 0})
        self.reset_sensor_input_flag = True

    @public_object(prefix="drivetrain")
    def reset_auto_config(self):
        self.autodrive_config_datastream.push({"max_y_speed": 7, "max_y_acceleration": 8, "y_tolerance": .3,
                                               "max_x_speed": 4, "max_x_acceleration": 6, "x_tolerance": .3,
                                               "max_rot_speed": 180, "max_rot_acceleration": 90, "rot_tolerance": 10})

    @public_object(prefix="drivetrain")
    def x_at_setpoint(self):
        x_pos = self.sensor_input_datastream.get().get("x_pos", 0)
        x_setpoint = self.autodrive_setpoint_datastream.get().get("x_pos", 0)
        x_tolerance = self.autodrive_config_datastream.get()["x_tolerance"]
        return abs(x_pos - x_setpoint) < x_tolerance

    @public_object(prefix="drivetrain")
    def y_at_setpoint(self):
        y_pos = self.sensor_input_datastream.get().get("y_pos", 0)
        y_setpoint = self.autodrive_setpoint_datastream.get().get("y_pos", 0)
        y_tolerance = self.autodrive_config_datastream.get()["y_tolerance"]
        return abs(y_pos - y_setpoint) < y_tolerance

    @public_object(prefix="drivetrain")
    def r_at_setpoint(self):
        r_pos = self.sensor_input_datastream.get().get("r_pos", 0)
        r_setpoint = self.autodrive_setpoint_datastream.get().get("r_pos", 0)
        r_tolerance = self.autodrive_config_datastream.get()["rot_tolerance"]
        return abs(r_pos - r_setpoint) < r_tolerance

    @public_object(prefix="drivetrain")
    @asyncio.coroutine
    def wait_for_x(self):
        while not self.x_at_setpoint():
            if not self.auto_drive_enabled:
                break
            yield from asyncio.sleep(.1)

    @public_object(prefix="drivetrain")
    @asyncio.coroutine
    def wait_for_y(self):
        while not self.y_at_setpoint():
            if not self.auto_drive_enabled:
                break
            yield from asyncio.sleep(.1)

    @public_object(prefix="drivetrain")
    @asyncio.coroutine
    def wait_for_r(self):
        while not self.r_at_setpoint():
            if not self.auto_drive_enabled:
                break
            yield from asyncio.sleep(.1)

    @public_object(prefix="drivetrain")
    @asyncio.coroutine
    def wait_for_xyr(self):
        yield from self.wait_for_x()
        yield from self.wait_for_y()
        yield from self.wait_for_r()

    auto_drive_enabled = False

    @public_object(prefix="drivetrain")
    def auto_drive_enable(self):
        self.auto_drive_enabled = True

    @public_object(prefix="drivetrain")
    def auto_drive_disable(self):
        self.auto_drive_enabled = False

    @yeti.autorun_coroutine
    @asyncio.coroutine
    def run_loop(self):

        last_cycle_timestamp = wpilib.Timer.getFPGATimestamp()
        self.reset_sensor_input_flag = True
        self.reset_jaguar_input_flag = True
        was_enabled = False
        while True:
            yield from asyncio.sleep(.05)

            #####################################
            # Sensor Input calculations

            # Handle value reset
            if self.reset_sensor_input_flag:
                last_x_pos = self.reset_x_pos
                last_y_pos = self.reset_y_pos
                last_r_pos = self.reset_r_pos
                if self.USE_GYRO:
                    self.gyro.reset()
                last_gyro_angle = 0
                self.reset_sensor_input_flag = False

            # Handle jaguar encoder value reset
            if self.reset_jaguar_input_flag:
                yield from asyncio.sleep(.1)
                last_wheel_positions = [c.getPosition() for c in self.motor_controllers]
                self.reset_jaguar_input_flag = False

            # Get delta time
            current_cycle_timestamp = wpilib.Timer.getFPGATimestamp()
            delta_time = current_cycle_timestamp - last_cycle_timestamp
            last_cycle_timestamp = current_cycle_timestamp

            # Track wheel movement and get distances and average speeds
            current_wheel_positions = [0, 0, 0, 0]
            current_wheel_positions[0] = self.motor_controllers[0].getPosition()
            current_wheel_positions[1] = self.motor_controllers[1].getPosition()
            current_wheel_positions[2] = -self.motor_controllers[2].getPosition()
            current_wheel_positions[3] = -self.motor_controllers[3].getPosition()

            delta_wheel_positions = [c - l for c, l in zip(current_wheel_positions, last_wheel_positions)]
            last_wheel_positions = current_wheel_positions[:]
            average_wheel_speeds = [d * self.ROT_TO_FEET / delta_time for d in delta_wheel_positions]

            # Calculate cartesian velocity of translation
            robot_y_speed = average_wheel_speeds[0]/4 + average_wheel_speeds[1]/4 + average_wheel_speeds[2]/4 + average_wheel_speeds[3]/4
            robot_x_speed = average_wheel_speeds[0]/4 - average_wheel_speeds[1]/4 - average_wheel_speeds[2]/4 + average_wheel_speeds[3]/4
            robot_r_speed = - average_wheel_speeds[0]/(4*self.mecanum_kinematic_k) - average_wheel_speeds[1]/(4*self.mecanum_kinematic_k)\
                           +average_wheel_speeds[2]/(4*self.mecanum_kinematic_k) + average_wheel_speeds[3]/(4*self.mecanum_kinematic_k)

            # If we are using the gyro, use it for r_speed
            if self.USE_GYRO:
                # Get gyro state and get average angle
                current_gyro_angle = -self.gyro.getAngle()
                delta_gyro_angle = current_gyro_angle - last_gyro_angle
                robot_r_speed = delta_gyro_angle / delta_time
                last_gyro_angle = current_gyro_angle

            # Get total global rotation from rotation speed
            r_pos = (robot_r_speed * delta_time) + last_r_pos

            # Convert robot-centric speeds to world-centric speeds
            x_speed, y_speed = rotate_vector(robot_x_speed, robot_y_speed, r_pos)
            r_speed = robot_r_speed

            # Calculate current xy position
            x_pos = (x_speed * delta_time) + last_x_pos
            y_pos = (y_speed * delta_time) + last_y_pos

            # If external inputs are configured, override values
            external_input_data = self.external_sensor_input_datastream.get()
            for name in external_input_data:
                external_input = external_input_data[name]
                if external_input.get("enabled", False):
                    x_override = external_input.get("x_pos", None)
                    y_override = external_input.get("y_pos", None)
                    r_override = external_input.get("r_pos", None)
                    if x_override is not None:
                        x_pos = x_override
                    if y_override is not None:
                        y_pos = y_override
                    if r_override is not None:
                        r_pos = r_override

            # Save values for next iteration
            last_x_pos = x_pos
            last_y_pos = y_pos
            last_r_pos = r_pos

            if self.DEBUG_NT_OUT:
                wpilib.SmartDashboard.putNumber("sensor_input_x_pos", x_pos)
                wpilib.SmartDashboard.putNumber("sensor_input_y_pos", y_pos)
                wpilib.SmartDashboard.putNumber("sensor_input_r_pos", r_pos)
                wpilib.SmartDashboard.putNumber("sensor_input_x_speed", x_speed)
                wpilib.SmartDashboard.putNumber("sensor_input_y_speed", y_speed)
                wpilib.SmartDashboard.putNumber("sensor_input_r_speed", r_speed)

            self.sensor_input_datastream.push({"x_pos": x_pos, "x_speed": x_speed,
                                               "y_pos": y_pos, "y_speed": y_speed,
                                               "r_pos": r_pos, "r_speed": r_speed,
                                               "timestamp": current_cycle_timestamp})

            ###############################
            # Auto drive calculation

            if self.auto_drive_enabled:

                # Get autonomous setpoints and config vars
                setpoint_data = self.autodrive_setpoint_datastream.get()
                config = self.autodrive_config_datastream.get()

                max_y_speed = config["max_y_speed"]
                max_y_accel = config["max_y_acceleration"]
                max_x_speed = config["max_x_speed"]
                max_x_accel = config["max_x_acceleration"]
                max_rot_speed = config["max_rot_speed"]
                max_rot_accel = config["max_rot_acceleration"]
                y_tolerance = config["y_tolerance"]
                x_tolerance = config["x_tolerance"]
                rot_tolerance = config["rot_tolerance"]

                setpoint_x_pos = setpoint_data.get("x_pos", 0)
                setpoint_y_pos = setpoint_data.get("y_pos", 0)
                setpoint_r_pos = setpoint_data.get("r_pos", 0)
                setpoint_x_speed = setpoint_data.get("x_speed", 0)
                setpoint_y_speed = setpoint_data.get("y_speed", 0)
                setpoint_r_speed = setpoint_data.get("r_speed", 0)

                # Calculate setpoint deltas
                x_pos_delta = setpoint_x_pos - x_pos
                y_pos_delta = setpoint_y_pos - y_pos
                r_pos_delta = setpoint_r_pos - r_pos

                # Use continuous_accel_filter to determine proper speed out.
                if abs(x_pos_delta) > x_tolerance:
                    x_speed_out = continuous_accel_filter(x_pos_delta, x_speed, setpoint_x_speed, max_x_accel, max_x_speed, delta_time)
                else:
                    x_speed_out = 0

                if abs(y_pos_delta) > y_tolerance:
                    y_speed_out = continuous_accel_filter(y_pos_delta, y_speed, setpoint_y_speed, max_y_accel, max_y_speed, delta_time)
                else:
                    y_speed_out = 0

                if abs(r_pos_delta) > rot_tolerance:
                    r_speed_out = continuous_accel_filter(r_pos_delta, r_speed, setpoint_r_speed, max_rot_accel, max_rot_speed, delta_time)
                else:
                    r_speed_out = 0

                # Convert from world-centric to robot-centric
                robot_x_out, robot_y_out = rotate_vector(x_speed_out, y_speed_out, -(r_pos + r_speed*delta_time))

                # Send values to drive loop
                self.control_datastream.push({"forward_fps": robot_y_out, "right_fps": robot_x_out, "ctrclockwise_dps": r_speed_out, "enable_esp": True})

                if not gamemode.is_autonomous():
                    self.auto_drive_enabled = False

            #######################################
            # Drive output

            if gamemode.is_enabled():
                if not was_enabled:
                    for jag in self.motor_controllers:
                        jag.setPID(self.JAG_P, self.JAG_I, self.JAG_D)
                was_enabled = True

                # Get control inputs
                control_data = self.control_datastream.get()
                forward_speed_in = control_data.get("forward_fps", 0)
                right_speed_in = control_data.get("right_fps", 0)
                ctrclockwise_speed_in = control_data.get("ctrclockwise_dps", 0)
                enable_esp = control_data.get("enable_esp", True)

                if self.DEBUG_NT_OUT:
                    wpilib.SmartDashboard.putNumber("drv_forward_speed", forward_speed_in)
                    wpilib.SmartDashboard.putNumber("drv_right_speed", right_speed_in)
                    wpilib.SmartDashboard.putNumber("drv_ctrclockwise_speed", ctrclockwise_speed_in)
                    wpilib.SmartDashboard.putBoolean("drv_enable_esp", enable_esp)

                if enable_esp:
                    self.set_speed_mode()

                    # Inverse kinematics to get mecanum values
                    front_left_out = forward_speed_in + right_speed_in - (ctrclockwise_speed_in * self.mecanum_kinematic_k)
                    rear_left_out = forward_speed_in - right_speed_in - (ctrclockwise_speed_in * self.mecanum_kinematic_k)
                    front_right_out = forward_speed_in - right_speed_in + (ctrclockwise_speed_in * self.mecanum_kinematic_k)
                    rear_right_out = forward_speed_in + right_speed_in + (ctrclockwise_speed_in * self.mecanum_kinematic_k)

                    # convert from fps to rpm
                    front_left_out *= self.FPS_TO_RPM
                    front_right_out *= self.FPS_TO_RPM
                    rear_left_out *= self.FPS_TO_RPM
                    rear_right_out *= self.FPS_TO_RPM

                    # Send to motors.
                    self.motor_controllers[0].set(front_left_out)
                    self.motor_controllers[1].set(rear_left_out)
                    self.motor_controllers[2].set(-front_right_out)
                    self.motor_controllers[3].set(-rear_right_out)

                else:
                    self.set_percent_mode()

                    forward_percentage = forward_speed_in / 10
                    right_percentage = right_speed_in / 10
                    ctrclockwise_percentage = ctrclockwise_speed_in / 270

                    # Inverse kinematics to get mecanum values
                    front_left_out = forward_percentage - ctrclockwise_percentage + right_percentage
                    front_right_out = forward_percentage + ctrclockwise_percentage - right_percentage
                    rear_left_out = forward_percentage - ctrclockwise_percentage - right_percentage
                    rear_right_out = forward_percentage + ctrclockwise_percentage + right_percentage

                    # Normalize values if we aren't using can + encoders
                    max_value = max(abs(front_left_out), abs(front_right_out), abs(rear_left_out), abs(rear_right_out))
                    if max_value > 1:
                        front_left_out /= max_value
                        front_right_out /= max_value
                        rear_left_out /= max_value
                        rear_right_out /= max_value

                    # Send to motors.
                    self.motor_controllers[0].set(front_left_out)
                    self.motor_controllers[1].set(rear_left_out)
                    self.motor_controllers[2].set(-front_right_out)
                    self.motor_controllers[3].set(-rear_right_out)
            else:
                was_enabled = False
                if self.auto_drive_enabled:
                    self.auto_drive_disable()
                else:
                    self.control_datastream.push({"forward_fps": 0, "right_fps": 0, "ctrclockwise_dps": 0, "enable_esp": True})
                    for controller in self.motor_controllers:
                        controller.set(0)

    # This is the constant, when multiplied by counterclockwise dps, results in the appropriate
    # right fps in order to rotate around a point 3ft in front of the robot.
    # It is essentially 3*pi/180
    STRAFE_TURN_CONST = 3 * math.pi / 180

    @gamemode.teleop_task
    @asyncio.coroutine
    def joystick_loop(self):
        last_reset_button = True
        while gamemode.is_teleop():

            # Ensure auto drive is disabled
            if self.auto_drive_enabled:
                self.auto_drive_disable()

            # Handle tracking reset button
            reset_button_val = self.joystick.getRawButton(5)
            if reset_button_val and not last_reset_button:
                self.reset_sensor_input()
            last_reset_button = reset_button_val

            forward_percentage = -self.joystick.getY()
            right_percentage = self.joystick.getX()
            ctrclockwise_percentage = -self.joystick.getZ()

            #Threshold values
            forward_percentage = threshold_value(forward_percentage, .10)
            right_percentage = threshold_value(right_percentage, .10)
            ctrclockwise_percentage = threshold_value(ctrclockwise_percentage, .10)

            if self.SQUARE_INPUTS:
                forward_percentage = signing_square(forward_percentage)
                right_percentage = signing_square(right_percentage)
                ctrclockwise_percentage = signing_square(ctrclockwise_percentage)

            # If button 10 is pressed, Zero output
            if self.joystick.getRawButton(10):
                forward_fps = 0
                right_fps = 0
                ctrclockwise_dps = 0
            # If button 1 is pressed, use faster speed setting
            elif self.joystick.getRawButton(1):
                forward_fps = forward_percentage * self.FAST_JOYSTICK_Y_FPS
                right_fps = right_percentage * self.FAST_JOYSTICK_X_FPS
                ctrclockwise_dps = ctrclockwise_percentage * self.FAST_JOYSTICK_R_DPS
            # Otherwise, use slower speed setting
            else:
                forward_fps = forward_percentage * self.SLOW_JOYSTICK_Y_FPS
                right_fps = right_percentage * self.SLOW_JOYSTICK_X_FPS
                ctrclockwise_dps = ctrclockwise_percentage * self.SLOW_JOYSTICK_R_DPS

            # If button 9 is pressed, rotate around point 3ft in front of bot
            if self.joystick.getRawButton(9):
                right_fps = self.STRAFE_TURN_CONST * ctrclockwise_dps

            # If button 2 is pressed, disable ESP
            enable_esp = not self.joystick.getRawButton(2)

            # Send values to drive loop
            self.control_datastream.push({"forward_fps": forward_fps, "right_fps": right_fps, "ctrclockwise_dps": ctrclockwise_dps, "enable_esp": enable_esp})

            yield from asyncio.sleep(.05)