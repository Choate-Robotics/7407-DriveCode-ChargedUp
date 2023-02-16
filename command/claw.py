import math
import config
from robotpy_toolkit_7407.command import SubsystemCommand
from subsystem import Claw

'''
reset command (0)
open command
close command
'''

class InitClaw(SubsystemCommand[Claw]):

    def __init__(self, subsystem: Claw):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.init()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

class ClawReset(SubsystemCommand[Claw]):

    def __init__(self, subsystem: Claw):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:   
        self.subsystem.zero()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return round(math.degrees(self.subsystem.get_angle())) == 0
    
    def end(self, interrupted: bool) -> None:
        pass

class ClawWrist(SubsystemCommand[Claw]):

    def __init__(self, subsystem: Claw, up: bool):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.up = up

    def initialize(self) -> None:
        self.subsystem.set_claw_ouput(config.claw_motor_speed)
        if self.up:
            self.subsystem.set_angle(math.pi / 2)
        else:
            self.subsystem.set_angle(0)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.up:
            return round(math.degrees(self.subsystem.get_angle())) == 90
        else:
            return round(math.degrees(self.subsystem.get_angle())) == 0

class ClawCrunch(SubsystemCommand[Claw]):

    def __init__(self, subsystem: Claw,  compress: bool):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.compress = compress

    def initialize(self) -> None:
        if self.compress:
            self.subsystem.open_claw()
        else:
            self.subsystem.close_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.subsystem.claw_compressed