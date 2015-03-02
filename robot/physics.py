from pyfrc.physics import core, drivetrains
import wpilib


class PhysicsEngine(core.PhysicsEngine):

    def update_sim(self, hal_data, now, tm_diff):
        front_left_wheel = hal_data["pwm"][2]["value"]
        rear_left_wheel = hal_data["pwm"][4]["value"]
        front_right_wheel = -hal_data["pwm"][1]["value"]
        rear_right_wheel = -hal_data["pwm"][3]["value"]

        vx, vy, vr = drivetrains.mecanum_drivetrain(rear_left_wheel, rear_right_wheel, front_left_wheel, front_right_wheel, .5, 14)
        self.physics_controller.vector_drive(vx, vy, vr, tm_diff)
