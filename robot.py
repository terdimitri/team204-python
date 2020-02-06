from vectors import Projective2d, Matrix2d
from math import tau, copysign, pi, cos, inf
import wpilib
import ctre


class SwerveModule:
    def __init__(self, drive_motor, turning_motor, encoder, full_turn, inverted=None):
        """At the start of program, it is expected that the drive motor will
        spin forwards, the turning motor will spin counter-clockwise, and the
        encoder will increase, given positive voltage. If they do not, set the
        appropriate `inverted` flags.
        """
        self.inverted = inverted
        if inverted is None:
            self.inverted = {
                    'encoder': False,
                    'turning_motor': False,
                    'drive_motor': False,
                }
        self.drive_motor = drive_motor
        self.turning_motor = self.turning_motor
        self.encoder = self.encoder
        self.full_turn = full_turn

    def set_turn(speed):
        """Sets the turning motor.
        """
        if inverted['turning_motor']:
            self.turning_motor.set(-speed)
        else:
            self.turning_motor.set(speed)

    def set_drive(speed):
        """Sets the driving motor.
        """
        if inverted['drive_motor']:
            self.drive_motor.set(-speed)
        else:
            self.drive_motor.set(speed)

    def current_angle(self):
        """Returns the current angle of the wheel based on the current encoder
        value.
        """
        if inverted['encoder']:
            return -self.encoder.encoder.get() / full_turn * tau
        else:
            return self.encoder.encoder.get() / full_turn * tau

    def set(vector, speed=1, p_coeff=0.02):
        """Sets the speed of the driving motor to the magnitude of the vector
        scaled by `speed`, and sets the turning motor to turn towards the
        desired angle.
        """
        # TODO: reverse wheel direction when appropriate
        if vector is None:
            stop()
        # proportional control for turning to specified angle
        current_angle = self.current_angle()
        target_angle = closest_to_mod(current_angle, self.full_turn//2)
        angle_delta = target_angle - current_angle
        turning_speed = -copysign(min(1, abs(p_coeff*angle_delta)), angle_delta)
        self.set_turn(turning_speed)
        # set speed of drive motor
        speed = speed * min(1, vector.magnitude)
        self.set_drive(speed)

    def stopMotor():
        """Stops both motors.
        """
        set_turn(0)
        set_drive(0)



class SwerveDrive(wpilib.drive.RobotDriveBase):
    def __init__(self, front_left, front_right, back_right, back_left, wheelbase, trackwidth):
        super().__init__()
        self.front_left = front_left
        self.front_right = front_right
        self.back_right = back_right
        self.back_left = back_left
        self.wheelbase = wheelbase
        self.trackwidth = trackwidth

    def set(forward, strafe_left, rotate):
        forward = self.applyDeadband(forward)
        strafe_left = self.applyDeadband(strafe_left)
        rotate = self.applyDeadband(rotate)

        c = Projective2d(forwards, strafe_left)
        # translation_intensity is the ratio of the magnitude of the joystick
        # position to the magnitude of the furthest possible joystick position
        # with the same argument
        translation_intensity = c.magnitude * cos((c.argument+pi4) % pi/2 + pi/4)
        c.argument += pi/2

        try:
            scalar = 1/rotate
        except ZeroDivisionError:
            scalar = inf

        c *= scalar
        self.cc_around()

    def cc_around(self, speed, c):
        """Makes the robot turn counter clockwise around `c`, which is in
        reference of the center of the robot, and in units of the distance from
        the origin to wheels at the given speed.
        """
        R = sqrt(self.trackwidth**2 + self.wheelbase**2)
        W = self.trackwidth / R
        L = self.wheelbase / R

        front_left_vec = Projective2d(L, W) - c
        front_right_vec = Projective2d(L, -W) - c
        back_right_vec = Projective2d(-L, -W) - c
        back_left_vec = Projective2d(-L, W) - c

        front_left_vec.argument += pi/2
        front_right_vec.argument += pi/2
        back_right_vec.argument += pi/2
        back_left_vec.argument += pi/2

        max_magnitude = max(front_left_vec.magnitude,
                             front_right_vec.magnitude,
                             back_right_vec.magnitude,
                             back_left_vec.magnitude)
        front_left_vec *= (1/max_magnitude)
        front_right_vec *= (1/max_magnitude)
        back_right_vec *= (1/max_magnitude)
        back_left_vec *= (1/max_magnitude)

        front_left.set(front_left_vec, speed)
        front_right.set(front_right_vec, speed)
        back_right.set(back_right_vec, speed)
        back_left.set(back_left_vec, speed)

        # Tell the watchdog that the motors have been set.
        self.feed()

    def stopMotor():
        front_left.stopMotor()
        front_right.stopMotor()
        back_right.stopMotor()
        back_left.stopMotor()

        # Tell the watchdog that the motors have been set.
        self.feed()



class OurRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.motors = [
                ctre.WPI_TalonSRX(11),
                ctre.WPI_TalonSRX(12),
                ctre.WPI_TalonSRX(13),
                ctre.WPI_TalonSRX(14),
                ctre.WPI_TalonSRX(21),
                ctre.WPI_TalonSRX(22),
                ctre.WPI_TalonSRX(23),
                ctre.WPI_TalonSRX(24)
            ]
        self.encoders = [
                wpilib.Encoder(0, 1),
                wpilib.Encoder(2, 3),
                wpilib.Encoder(4, 5),
                wpilib.Encoder(6, 7)
            ]

    def teleopPeriodic(self):
        pass
