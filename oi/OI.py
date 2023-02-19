import math

from commands2 import InstantCommand, ParallelCommandGroup, SequentialCommandGroup
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

        # Keymap.Intake.INTAKE_ENABLE.whenPressed(
        #     command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(45))
        # )
        #
        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0))
        # )
        #
        # Keymap.Claw.ENGAGE_CLAW.whenPressed(
        #     command.Target(
        #         Robot.drivetrain,
        #         Robot.arm,
        #         Robot.grabber,
        #         Sensors.odometry,
        #         pose=Pose2d(14.44, 1.05, 0),  # 8.4 2.3 0
        #         arm_angle=math.radians(-44.78),
        #         arm_length=0.55,
        #         wrist_angle=math.radians(-27.09),
        #         wrist_enabled=True,
        #     )
        # )
        #
        # pick_up = (-103.5, 0.099, -20.53)
        # score_mid = (-44.78, 0.55, -27.09)
        # score_high = (-47.7, 1.04, -18.61)
        #
        # Keymap.Claw.ENGAGE_CLAW.whenReleased(
        #     InstantCommand(
        #         lambda: commands2.CommandScheduler.getInstance().schedule(
        #             commands=[
        #                 command.DriveSwerveCustom(Robot.drivetrain),
        #                 SequentialCommandGroup(
        #                     command.SetGrabber(Robot.grabber, 0, False),
        #                     WaitCommand(0.4),
        #                     InstantCommand(lambda: Robot.grabber.set_output(0)),
        #                     command.SetArm(Robot.arm, 0, 0),
        #                 ),
        #                 command.IntakeDisable(Robot.intake),
        #             ]
        #         )
        #     )
        # )

        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     command.SetArm(Robot.Arm, 0, 0, 0).andThen(
        #         WaitCommand(0).andThen(command.IntakeDisable(Robot.intake))
        #     )
        # )
        # Keymap.Intake.INTAKE_ENABLE.whenPressed(
        #     ParallelCommandGroup(
        #         command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(45)),
        #         command.SetGrabber(Robot.grabber, wrist_angle=math.radians(-45), claw_active=True)
        #     )
        # )
        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     ParallelCommandGroup(
        #         command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0)),
        #         command.SetGrabber(Robot.grabber, wrist_angle=math.radians(0), claw_active=False)
        #     )
        # )

        # Keymap.Intake.INTAKE_ENABLE.whenPressed(
        #     command.setElevator(Robot.arm, .3)
        # )
        #
        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     command.setElevator(Robot.arm, 0)
        # )

        # Keymap.Claw.ENGAGE_CLAW.onTrue(InstantCommand(lambda: Robot.arm.engage_claw()))
        # Keymap.Claw.ENGAGE_CLAW.onFalse(
        #     InstantCommand(lambda: Robot.arm.disengage_claw())
        #
        # )

        pick_up = (-100, 0.099, -20.53)
        score_mid = (-44.78, 0.55, -27.09)
        score_high = (-47.7, 1.04, -18.61)

        Keymap.Intake.PICK_UP_ARM.whenPressed(
            ParallelCommandGroup(
                command.SetArm(
                    Robot.arm, distance=0.099, shoulder_angle=math.radians(-100)
                ),
                command.SetGrabber(
                    Robot.grabber, wrist_angle=math.radians(-20.53), claw_active=True
                ),
            )
        )

        Keymap.Intake.PICK_UP_ARM.whenReleased(
            ParallelCommandGroup(
                command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0)),
                command.SetGrabber(
                    Robot.grabber, wrist_angle=math.radians(0), claw_active=False
                ),
            )
        )

        Keymap.Intake.DROP_OFF_ARM.whenPressed(
            ParallelCommandGroup(
                command.SetArm(
                    Robot.arm, distance=0.55, shoulder_angle=math.radians(-44.78)
                ),
                command.SetGrabber(
                    Robot.grabber, wrist_angle=math.radians(-27.09), claw_active=False
                ),
            )
        )

        Keymap.Intake.DROP_OFF_ARM.whenReleased(
            ParallelCommandGroup(
                command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0)),
                command.SetGrabber(
                    Robot.grabber, wrist_angle=math.radians(0), claw_active=False
                ),
            )
        )

        Keymap.Intake.GRABBER_PICK.whenPressed(
            InstantCommand(lambda: Robot.grabber.engage_claw())
        )

        Keymap.Intake.GRABBER_PICK.whenReleased(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw())
            )
        )

        Keymap.Intake.GRABBER_SCORE.whenPressed(
            InstantCommand(lambda: Robot.grabber.open_claw())
        )

        Keymap.Intake.GRABBER_SCORE.whenReleased(
            InstantCommand(lambda: Robot.grabber.disengage_claw())
        )
