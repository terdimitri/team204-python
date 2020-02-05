from vectors import Projective2d, Matrix2d
from math import tau, copysign
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
        self.turning_motor.set(speed)

    def set_drive(speed):
        self.drive_motor.set(speed)

    def current_angle(self):
        return self.encoder.encoder.get() / full_turn * tau

    def set(vector, speed=1, p_coeff=0.02):
        # set speed of drive motor
        speed = speed * vector.magnitude
        self.set_drive(speed)
        # proportional control for turning to specified angle
        current_angle = self.current_angle()
        angle_target = closest_to_mod(current_angle, self.full_turn/2)
        angle_delta = closest_to_mod(vector.argument, self.current_angle) - self.current_angle()
        turning_speed = -copysign(min(1, abs(p_coeff*angle_delta)), angle_delta)
        self.set_turn(turning_speed)






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
