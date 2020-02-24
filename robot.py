"""This is a module docstring"""

from math import tau, pi, cos, inf, sqrt
import wpilib
import wpilib.drive
from wpilib.interfaces import GenericHID
import ctre

from vectors import Projective2d
import simple_pid


def closest_to_mod(a, c, n):
    """Return b congruent to a modulo n such that abs(b-c) is minimized.
    """
    delta = (a - c) % n
    if n - delta < delta:
        delta -= n
    return c + delta


class SwerveModule:
    full_turn = 415

    def __init__(self, drive_motor, turning_motor, encoder, inverted=None):
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
        self.turning_motor = turning_motor
        self.encoder = encoder

        # PID controller
        self.turner = simple_pid.PID(Kp=1.0, Ki=0.0, Kd=0.0,        # p, i, and d coefficients
                                     sample_time=None,              # update values every loop
                                     output_limits=(-1.0, 1.0),
                                     input_limits=(0, pi),
                                     wrap_around=True,              # input is circular
                                     auto_mode=True)                # enable the pid

    def set_turn(self, speed):
        """Sets the turning motor.
        """
        if self.inverted['turning_motor']:
            self.turning_motor.set(-speed)
        else:
            self.turning_motor.set(speed)

    def set_drive(self, speed):
        """Sets the driving motor.
        """
        if self.inverted['drive_motor']:
            self.drive_motor.set(-speed)
        else:
            self.drive_motor.set(speed)

    def current_angle(self):
        """Returns the current angle of the wheel based on the current encoder
        value.
        """
        if self.inverted['encoder']:
            return -self.encoder.encoder.get() / self.full_turn * tau
        return self.encoder.get() / self.full_turn * tau

    def angle_to_encoder(self, angle):
        """Converts the given angle to a possible encoder value"""
        return round(angle/tau*self.full_turn)

    def angle_error(self, vector):
        """Returns distance between current angle and argument of the vector."""
        current_angle = self.current_angle()
        return abs(closest_to_mod(vector.argument, current_angle, pi) - current_angle)

    def set(self, vector, speed=1):
        """Sets the speed of the driving motor to the magnitude of the vector
        scaled by `speed`, and sets the turning motor to turn towards the
        desired angle.
        """
        if vector is None:
            self.stopMotor()

        self.set_turn(self.turner(self.current_angle()))

        # If the wheel is backwards, spin it in reverse
        reverse = 1
        if self.angle_error(vector) > pi/2:     # angle error is very big, wheel is backwards
            reverse = -1

        # set speed of drive motor
        speed = speed * min(1, vector.magnitude)
        self.set_drive(reverse*speed)

    def stopMotor(self):
        """Stops both motors.
        """
        self.set_turn(0)
        self.set_drive(0)



class SwerveDrive(wpilib.drive.RobotDriveBase):
    angle_tolerance = tau/20
    def __init__(self, front_left, front_right, back_right, back_left, wheelbase, trackwidth):
        super().__init__()
        self.front_left = front_left
        self.front_right = front_right
        self.back_right = back_right
        self.back_left = back_left
        self.wheelbase = wheelbase
        self.trackwidth = trackwidth

        self.front_left_vec = Projective2d(0, 0)
        self.front_right_vec = Projective2d(0, 0)
        self.back_right_vec = Projective2d(0, 0)
        self.back_left_vec = Projective2d(0, 0)

    def set(self, forward, strafe_left, rotate):
        forward = self._applyDeadband(forward, self._m_deadband)
        strafe_left = self._applyDeadband(strafe_left, self._m_deadband)
        rotate = self._applyDeadband(rotate, self._m_deadband)

        center = Projective2d(forward, strafe_left)
        # translation_intensity is the ratio of the magnitude of the joystick
        # position to the magnitude of the furthest possible joystick position
        # with the same argument
        translation_intensity = center.magnitude * cos((center.argument+pi/4) % pi/2 + pi/4)
        center.magnitude = translation_intensity
        center.argument += pi/2

        try:
            scalar = 1/rotate
        except ZeroDivisionError:
            scalar = inf
        center *= scalar

        # max_possible_rotate is the maximum rotate input that would be able to
        # result in the same center
        try:
            max_possible_rotate = 1 / center.magnitude
        except ZeroDivisionError:
            max_possible_rotate = inf
        max_possible_rotate = min(1, max_possible_rotate)

        try:
            speed = rotate / max_possible_rotate
        except ZeroDivisionError:
            speed = translation_intensity

        self.cc_around(speed, center)

    def cc_around(self, speed, c):
        """Makes the robot turn counter clockwise around `c`, which is in
        reference of the center of the robot, and in units of the distance from
        the origin to wheels at the given speed.
        """
        R = sqrt(self.trackwidth**2 + self.wheelbase**2)
        W = self.trackwidth / R
        L = self.wheelbase / R

        self.front_left_vec = Projective2d(L, W) - c
        self.front_right_vec = Projective2d(L, -W) - c
        self.back_right_vec = Projective2d(-L, -W) - c
        self.back_left_vec = Projective2d(-L, W) - c

        self.front_left_vec.argument += pi/2
        self.front_right_vec.argument += pi/2
        self.back_right_vec.argument += pi/2
        self.back_left_vec.argument += pi/2

        max_magnitude = max(self.front_left_vec.magnitude,
                            self.front_right_vec.magnitude,
                            self.back_right_vec.magnitude,
                            self.back_left_vec.magnitude)
        try:
            self.front_left_vec *= (1/max_magnitude)
            self.front_right_vec *= (1/max_magnitude)
            self.back_right_vec *= (1/max_magnitude)
            self.back_left_vec *= (1/max_magnitude)
        except ZeroDivisionError:
            self.front_left_vec.magnitude = 1
            self.front_right_vec.magnitude = 1
            self.back_right_vec.magnitude = 1
            self.back_left_vec.magnitude = 1

        max_angle_error = max(self.front_left.angle_error(self.front_left_vec),
                              self.front_right.angle_error(self.front_right_vec),
                              self.back_right.angle_error(self.back_right_vec),
                              self.back_left.angle_error(self.back_left_vec))

        if max_angle_error <= self.angle_tolerance:
            self.front_left.set(self.front_left_vec, 0)
            self.front_right.set(self.front_right_vec, 0)
            self.back_right.set(self.back_right_vec, 0)
            self.back_left.set(self.back_left_vec, 0)
        else:
            self.front_left.set(self.front_left_vec, speed)
            self.front_right.set(self.front_right_vec, speed)
            self.back_right.set(self.back_right_vec, speed)
            self.back_left.set(self.back_left_vec, speed)

        # Tell the watchdog that the motors have been set.
        self.feed()

    def stopMotor(self):
        """Stops all motors.
        """
        self.front_left.stopMotor()
        self.front_right.stopMotor()
        self.back_right.stopMotor()
        self.back_left.stopMotor()

        # Tell the watchdog that the motors have been set.
        self.feed()

    def getDescription(self) -> str:
        return "SwerveDrive"


class OurRobot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

        self.drive_motors = {
            'front left':  ctre.WPI_TalonSRX(24),
            'front right': ctre.WPI_TalonSRX(14),
            'back right':  ctre.WPI_TalonSRX(12),
            'back left':   ctre.WPI_TalonSRX(22),
            }
        self.turning_motors = {
            'front left':  ctre.WPI_TalonSRX(23),
            'front right': ctre.WPI_TalonSRX(13),
            'back right':  ctre.WPI_TalonSRX(11),
            'back left':   ctre.WPI_TalonSRX(21),
            }
        self.encoders = {
            'front left':  wpilib.Encoder(6, 7),
            'front right': wpilib.Encoder(4, 5),
            'back right':  wpilib.Encoder(0, 1),
            'back left':   wpilib.Encoder(2, 3)
            }

        # 0 is the left hand, 1 is the right hand
        self.controller = wpilib.XboxController(0)

        self.drive = SwerveDrive(
            SwerveModule(self.drive_motors['front left'],
                         self.turning_motors['front left'],
                         self.encoders['front left']),
            SwerveModule(self.drive_motors['front right'],
                         self.turning_motors['front right'],
                         self.encoders['front right']),
            SwerveModule(self.drive_motors['back right'],
                         self.turning_motors['back right'],
                         self.encoders['back right']),
            SwerveModule(self.drive_motors['back left'],
                         self.turning_motors['back left'],
                         self.encoders['back left']),
            20, 25.5
        )

    def robotInit(self):
        self.logger.info("Hello world!")
    def robotPeriodic(self):
        pass

    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        forward = -self.controller.getY(GenericHID.Hand.kLeftHand)
        left = -self.controller.getX(GenericHID.Hand.kLeftHand)
        rotate_cc = -self.controller.getX(GenericHID.Hand.kRightHand)

        for k in self.encoders:
            wpilib.SmartDashboard.putString(k + ' encoder', str(self.encoders[k].get()))

        self.drive.set(forward, left, rotate_cc)

    def testInit(self):
        pass
    def testPeriodic(self):
        pass

    def disabledInit(self):
        pass
    def disabledPeriodic(self):
        pass


if __name__ == '__main__':
    wpilib.run(OurRobot)
