import math

import commands2
import wpilib
from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand,
)
from robotpy_toolkit_7407.utils import logger

import command
import config
from oi.keymap import Controllers, Keymap
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
            InstantCommand(
                lambda: Robot.drivetrain.gyro.reset_angle(Robot.drivetrain.start_angle)
            )
        )

        Keymap.Drivetrain.RESET_ODOMETRY.whenPressed(
            InstantCommand(
                lambda: Robot.drivetrain.reset_odometry(Robot.drivetrain.start_pose)
            )
        )

        Keymap.Drivetrain.X_MODE.whenPressed(
            InstantCommand(lambda: Robot.drivetrain.x_mode())
        )

        Keymap.Drivetrain.SLOW_FORWARD.whenPressed(
            command.DrivetrainScoreSlow(Robot.drivetrain)
        ).whenReleased(command.DrivetrainRegular(Robot.drivetrain, Sensors.odometry))

        # Keymap.Drivetrain.SLOW_REVERSE.whenPressed(
        #     command.DrivetrainScoreFront(Robot.drivetrain, Sensors.odometry)
        # ).whenReleased(command.DrivetrainRegular(Robot.drivetrain, Sensors.odometry))

        Keymap.Climber.DEPLOY.whenPressed(command.ClimberDeploy(Robot.climber))

        Keymap.Climber.RESET.whenPressed(command.ClimberRetract(Robot.climber))

        Keymap.Climber.UNCLIMB.whenPressed(command.ClimberUnpivot(Robot.climber))

        Keymap.Climber.CLIMB.whenPressed(command.ClimberPivot(Robot.climber))

        # TARGETING
        Keymap.Targeting.TARGETING_PICKUP.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["pickup"],
            )
        )

        Keymap.Targeting.TARGETING_PICKUP.whenReleased(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                InstantCommand(
                    lambda: Controllers.OPERATOR_CONTROLLER.setRumble(
                        wpilib.Joystick.RumbleType.kBothRumble, 0
                    )
                ),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard_pickup"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_LOW.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["low"],
            )
        ).whenReleased(
            SequentialCommandGroup(
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
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
                Sensors.odometry,
                target=config.scoring_locations["middle"],
            )
        )

        Keymap.Targeting.TARGETING_MIDDLE.whenReleased(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
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
                Sensors.odometry,
                target=config.scoring_locations["high"],
            )
        )

        def gyro_angle_calc():
            gyro_angle = Sensors.gyro.get_robot_heading() % (math.pi * 2)
            gyro_angle = math.degrees(
                math.atan2(math.sin(gyro_angle), math.cos(gyro_angle))
            )

            if -90 < gyro_angle < 90:
                return 1
            else:
                return -1

        def set_claw_angle(angle):
            config.grabber_target_angle = angle

        Keymap.Claw.DROP_CLAW.whenPressed(
            InstantCommand(
                lambda: set_claw_angle(math.radians(30) * gyro_angle_calc() * -1)
            )
        ).whenReleased(
            InstantCommand(lambda: set_claw_angle(math.radians(25) * gyro_angle_calc()))
        )

        Keymap.Targeting.TARGETING_HIGH.whenReleased(
            SequentialCommandGroup(
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_DOUBLE_STATION.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["double_station"],
            )
        )

        Keymap.Targeting.TARGETING_DOUBLE_STATION.whenReleased(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                WaitCommand(0.3),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_CUBE_INTAKE_CLAW.whenPressed(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.engage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["cube_intake"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_CUBE_INTAKE_CLAW.whenReleased(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                InstantCommand(
                    lambda: Controllers.OPERATOR_CONTROLLER.setRumble(
                        wpilib.Joystick.RumbleType.kBothRumble, 0
                    )
                ),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard_pickup"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_CUBE_INTAKE.whenPressed(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.engage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["cube_intake_no_grab"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_CUBE_INTAKE.whenReleased(
            SequentialCommandGroup(
                InstantCommand(
                    lambda: Controllers.OPERATOR_CONTROLLER.setRumble(
                        wpilib.Joystick.RumbleType.kBothRumble, 0
                    )
                ),
                InstantCommand(lambda: Robot.grabber.open_claw()),
                InstantCommand(lambda: Robot.grabber.set_output(0)),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard_pickup"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_EJECT_INTAKE.whenPressed(
            InstantCommand(lambda: Robot.intake.enable_intake_motor_max(True))
        )

        Keymap.Targeting.TARGETING_EJECT_INTAKE.whenReleased(
            InstantCommand(lambda: Robot.intake.disable_intake_motor())
        )

        Keymap.Claw.OPEN_CLAW_DRIVER.whenPressed(
            InstantCommand(lambda: Robot.grabber.open_claw()).andThen(
                lambda: print("DONE")
            )
        )

        Keymap.Claw.OPEN_CLAW_OPERATOR.whenPressed(
            InstantCommand(lambda: Robot.grabber.open_claw()).andThen(
                lambda: print("DONE")
            )
        ).whenReleased(
            InstantCommand(lambda: Robot.grabber.close_claw()).andThen(
                lambda: print("DONE")
            )
        )

        Keymap.Claw.RUN_CLAW_UP.whenPressed(
            InstantCommand(lambda: Robot.grabber.set_output(-0.4))
        ).whenReleased(InstantCommand(lambda: Robot.grabber.set_output(0)))

        Keymap.Claw.RUN_CLAW_DOWN.whenPressed(
            InstantCommand(lambda: Robot.grabber.set_output(0.4))
        ).whenReleased(InstantCommand(lambda: Robot.grabber.set_output(0)))

        Keymap.Drivetrain.AUTO_ROUTE.whenPressed(
            command.TargetDrivetrain(
                Sensors.odometry,
            )
        ).whenReleased(
            InstantCommand(
                lambda: commands2.CommandScheduler.getInstance().schedule(
                    command.DriveSwerveCustom(Robot.drivetrain)
                )
            )
        )

        def invert_elevator():
            config.elevator_voltage_inverted = not config.elevator_voltage_inverted

        Keymap.Debug.INVERT_ELEVATOR.whenPressed(
            InstantCommand(lambda: invert_elevator())
        )

        def edit_current_scoring_position(key: int | None):
            config.current_scoring_position = key

        Keymap.Scoring.ONE.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(0))
        )

        Keymap.Scoring.TWO.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(1))
        )

        Keymap.Scoring.THREE.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(2))
        )

        Keymap.Scoring.FOUR.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(3))
        )

        Keymap.Scoring.FIVE.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(4))
        )

        Keymap.Scoring.SIX.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(5))
        )

        Keymap.Scoring.SEVEN.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(6))
        )

        Keymap.Scoring.EIGHT.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(7))
        )

        Keymap.Scoring.NINE.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(8))
        )

        Keymap.Scoring.DEL.whenPressed(
            InstantCommand(lambda: edit_current_scoring_position(None))
        )
