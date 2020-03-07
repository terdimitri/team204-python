import wpilib
import wpilib.drive
from wpilib.interfaces import GenericHID
import ctre

LEFT = GenericHID.Hand.kLeftHand
RIGHT = GenericHID.Hand.kRightHand

class MyRobot(wpilib.IterativeRobot):
    MOTOR_PINS = {
        'left1': 0,
        'left2': 0,
        'right1': 0,
        'right2': 0,
        'shooter': 0,
        'grabber': 0,
        'intermediate': 0,
        'elevator': 0,
        'winch': 0
    }

    def robotInit(self):
        self.elevator_up = False

        # contoller
        self.controller = wpilib.XboxController(0)

        # motors
        self.motors = {name: ctre.WPI_TalonSRX(pin) for name, pin in self.MOTOR_PINS.items()}

        # drive motors
        self.left_side = wpilib.SpeedControllerGroup(self.motors['left1'], self.motors['left2'])
        self.right_side = wpilib.SpeedControllerGroup(self.motors['right2'], self.motors['right1'])
        self.drive = wpilib.drive.DifferentialDrive(self.left_side, self.right_side)

        # climbing motors
        self.winch = self.motors['winch']
        self.elevator = self.motors['elevator']

        # ball handling motors
        self.shooter = self.motors['shooter']
        self.grabber = self.motors['grabber']
        self.intermediate = self.motors['intermediate']

    def teleopPeriodic(self):
        # drive the robot
        self.drive.tankDrive(self.controller.getY(LEFT), self.controller.getY(RIGHT))

        # right bumper for shooting
        if self.controller.getBumper(RIGHT):
            self.shooter.set(1)
            self.intermediate.set(self.controller.getTriggerAxis(RIGHT))
        else:
            self.shooter.set(0)
            self.intermediate.set(0)

        # left bumper for picking up
        if self.controller.getBumper(LEFT):
            self.grabber.set(1)
            self.intermediate.set(self.controller.getTriggerAxis(RIGHT))
        else:
            self.grabber.set(0)
            self.intermediate.set(0)

        # elevator
        if self.controller.getYButtonPressed():
            self.elevator_up = not self.elevator_up
        if self.elevator_up:
            self.elevator.set(1)
        else:
            self.elevator.set(0)

        # winch
        if self.controller.getAButton():
            self.winch.set(1)
        else:
            self.winch.set(0)


if __name__ == '__main__':
    wpilib.run(MyRobot)
