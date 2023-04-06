from robotpy_toolkit_7407.command import SubsystemCommand

import constants
from robot_systems import Robot
from subsystem import Climber


class ClimberZero(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        Robot.climber.zero()
        Robot.climber.climber_disable()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=True):
        pass


class OperatorControl(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        if not Robot.climber.climber_active:
            Robot.climber.climber_deploy()
        else:
            Robot.climber.climber_disable()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False):
        pass


class ClimberDeploy(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        Robot.climber.climber_deploy()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False):
        pass


class ClimberRetract(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        Robot.climber.climber_reset()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False):
        pass


class ClimberPivot(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        Robot.climber.pivot()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return self.subsystem.is_climbed()

    def end(self, interrupted=False):
        ...


class ClimberUnpivot(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.unlatch_target = 0

    def initialize(self):
        self.unlatch_target = (
            self.subsystem.get_motor_rotations() + constants.climber_unlatch_extension
        )

        if self.subsystem.climber_active and self.subsystem.pivoted:
            self.subsystem.latch_disable()
            self.subsystem.set_motor_rotations(self.unlatch_target)

    def execute(self):
        ...

    def isFinished(self) -> bool:
        return abs(self.subsystem.get_motor_rotations() - self.unlatch_target) < 3

    def end(self, interrupted=False):
        if interrupted:
            ...
        else:
            self.subsystem.unpivot()
