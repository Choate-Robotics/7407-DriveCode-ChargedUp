import math

from commands2 import InstantCommand, WaitCommand
from robotpy_toolkit_7407.utils import logger

import command
from oi.keymap import Keymap
from robot_systems import Robot

logger.info("Hi, I'm OI!")


class OI:
    @staticmethod
    def init() -> None:
        pass

    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")

        # Keymap.Arm.REZERO_ELEVATOR.whenPressed(command.ZeroArm())

        # Keymap.Arm.ENGAGE_CLAW().whenPressed(engageClaw()).whenReleased(disEngageClaw())

        # Keymap.Intake.INTAKE_ENABLE.whenPressed(
        #     command.SetArm(Robot.Arm, 0, math.radians(0), math.radians(0), False)
        #     # command.IntakeEnable(Robot.intake).andThen(
        #     #     WaitCommand(0).andThen(command.SetArm(Robot.Arm, 0, math.radians(45), math.radians(45), False))
        #     # )
        # )

        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     command.SetArm(Robot.Arm, 0, 0, 0).andThen(
        #         WaitCommand(0).andThen(command.IntakeDisable(Robot.intake))
        #     )
        # )
        Keymap.Intake.INTAKE_ENABLE.whileHeld(
            command.SetArm(Robot.Arm, 0, math.radians(45), -math.radians(0)).andThen(
                command.EngageClaw(Robot.Arm)
            )
        )
        
        # Keymap.Intake.INTAKE_ENABLE.whileHeld(
        #     command.setShoulderRotation(Robot.Arm, math.radians(90)).andThen(
        #         command.EngageClaw(Robot.Arm)
        #     )
        # )

        Keymap.Claw.ENGAGE_CLAW.onTrue(InstantCommand(lambda: Robot.Arm.engage_claw()))
        Keymap.Claw.ENGAGE_CLAW.onFalse(
            InstantCommand(lambda: Robot.Arm.disengage_claw())
        )
