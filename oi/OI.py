import commands2
from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand,
)
from robotpy_toolkit_7407.utils import logger

import command
import config
from oi.keymap import Keymap
from robot_systems import Robot, Sensors

logger.info("Hi, I'm OI!")


class OI:
    @staticmethod
    def init() -> None:
        pass

    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")

        # DRIVETRAIN
        Keymap.Drivetrain.RESET_GYRO.whenPressed(
            InstantCommand(lambda: Robot.drivetrain.gyro.reset_angle(0))
        )

        Keymap.Drivetrain.RESET_ODOMETRY.whenPressed(
            InstantCommand(
                lambda: Robot.drivetrain.reset_odometry(Robot.drivetrain.start_pose)
            )
        )

        # TARGETING
        Keymap.Targeting.TARGETING_PICKUP.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Robot.drivetrain,
                Sensors.odometry,
                target=config.scoring_locations["pickup"],
            )
        )

        Keymap.Targeting.TARGETING_PICKUP.whenReleased(
            SequentialCommandGroup(
                InstantCommand(
                    lambda: commands2.CommandScheduler.getInstance().schedule(
                        command.DriveSwerveCustom(Robot.drivetrain)
                    )
                ),
                InstantCommand(lambda: Robot.grabber.close_claw()),
                WaitCommand(config.scoring_locations["pickup"].claw_wait_time),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Robot.drivetrain,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_MIDDLE.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Robot.drivetrain,
                Sensors.odometry,
                target=config.scoring_locations["middle"],
            )
        )

        Keymap.Targeting.TARGETING_MIDDLE.whenReleased(
            SequentialCommandGroup(
                InstantCommand(
                    lambda: commands2.CommandScheduler.getInstance().schedule(
                        command.DriveSwerveCustom(Robot.drivetrain)
                    )
                ),
                InstantCommand(lambda: Robot.grabber.close_claw()),
                WaitCommand(config.scoring_locations["middle"].claw_wait_time),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Robot.drivetrain,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_HIGH.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Robot.drivetrain,
                Sensors.odometry,
                target=config.scoring_locations["high"],
            )
        )

        Keymap.Targeting.TARGETING_HIGH.whenReleased(
            SequentialCommandGroup(
                InstantCommand(
                    lambda: commands2.CommandScheduler.getInstance().schedule(
                        command.DriveSwerveCustom(Robot.drivetrain)
                    )
                ),
                InstantCommand(lambda: Robot.grabber.close_claw()),
                WaitCommand(config.scoring_locations["high"].claw_wait_time),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Robot.drivetrain,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
        )

        Keymap.Claw.OPEN_CLAW.whenPressed(
            InstantCommand(lambda: Robot.grabber.open_claw())
        )

        Keymap.Claw.OPEN_CLAW.whenReleased(
            InstantCommand(lambda: Robot.grabber.close_claw())
        )
