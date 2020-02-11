from math import tau, copysign, pi, cos, inf, sqrt
import wpilib
import wpilib.drive
from wpilib.interfaces import GenericHID
import ctre

from vectors import Projective2d


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

        self.turn_controller = wpilib.PIDController()

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

    def pidWrite(self, value):
        """Set turn speed to value. Used for pid"""
        self.set_turn(value)

    def pidGet(self):
        """Gives the wheel direction. used for pid"""
        return self.encoder.get() % (self.full_turn//2)

    def set(self, vector, speed=1):
        """Sets the speed of the driving motor to the magnitude of the vector
        scaled by `speed`, and sets the turning motor to turn towards the
        desired angle.
        """
        if vector is None:
            self.stopMotor()
        # proportional control for turning to specified angle
        current_angle = self.current_angle()
        target_angle = closest_to_mod(vector.argument, current_angle, pi)
        #angle_delta = target_angle - current_angle
        #encoder_delta = self.angle_to_encoder(angle_delta)
        #turning_speed = -copysign(min(1, abs(p_coeff*encoder_delta)), encoder_delta)
        #self.set_turn(turning_speed)

        # if target_angle is the argument backwards, reverse the speed
        reverse = 1
        if abs((vector.argument - target_angle) % tau - pi) < pi/2:
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
        try:
            front_left_vec *= (1/max_magnitude)
            front_right_vec *= (1/max_magnitude)
            back_right_vec *= (1/max_magnitude)
            back_left_vec *= (1/max_magnitude)
        except ZeroDivisionError:
            front_left_vec.magnitude = 1
            front_right_vec.magnitude = 1
            back_right_vec.magnitude = 1
            back_left_vec.magnitude = 1

        max_angle_error = max(self.front_left.angle_error(front_left_vec),
                              self.front_right.angle_error(front_right_vec),
                              self.back_right.angle_error(back_right_vec),
                              self.back_left.angle_error(back_left_vec))

        if max_angle_error <= self.angle_tolerance:
            self.front_left.set(front_left_vec, 0)
            self.front_right.set(front_right_vec, 0)
            self.back_right.set(back_right_vec, 0)
            self.back_left.set(back_left_vec, 0)
        else:
            self.front_left.set(front_left_vec, speed)
            self.front_right.set(front_right_vec, speed)
            self.back_right.set(back_right_vec, speed)
            self.back_left.set(back_left_vec, speed)

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
            wpilib.SmartDashboard.putString(k, str(self.encoders[k].get()))

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
