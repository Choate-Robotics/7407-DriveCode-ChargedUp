import math

from robotpy_toolkit_7407.command import SubsystemCommand

from robot_systems import Robot
from subsystem import Intake


class IntakeEnable(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, intake_reversed: bool = False):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.reversed = intake_reversed

    def initialize(self):
        Robot.intake.intake_enable(intake_reversed=self.reversed)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False):
        pass
        # InstantCommand(command.ZeroArm(Robot.Arm))


class IntakeDisable(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, arm_angle_threshold=math.radians(60)):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.arm_angle_threshold = arm_angle_threshold
        self.finished = False

    def initialize(self):
        pass

    def execute(self):
        if Robot.arm.get_rotation() < self.arm_angle_threshold:
            Robot.intake.intake_disable()
            self.finished = True

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted=False):
        pass
    
