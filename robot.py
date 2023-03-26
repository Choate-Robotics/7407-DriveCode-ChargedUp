import math

import commands2
import wpilib
from commands2 import InstantCommand, SequentialCommandGroup
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

import autonomous
import command
import config
import constants
from autonomous.auto_routine import AutoRoutine
from oi.OI import OI
from robot_systems import Pneumatics, Robot, Sensors
from sensors import FieldOdometry, PV_Cameras
from utils import logger


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.pv_selection: wpilib.SendableChooser | None = None
        self.auto_selection: wpilib.SendableChooser | None = None
        self.can_reset_climb: wpilib.SendableChooser | None = None

    def robotInit(self):
        period = 0.05
        commands2.CommandScheduler.getInstance().setPeriod(period)
        Pneumatics.compressor.enableAnalog(90, 120)
        Robot.arm.init()
        Robot.drivetrain.init()
        Robot.intake.init()
        Robot.grabber.init()
        Robot.climber.init()

        Sensors.pv_controller = None
        Sensors.odometry = FieldOdometry(Robot.drivetrain, None)
        Sensors.gyro = Robot.drivetrain.gyro

        # SmartDashboard.putNumber("ELEVATOR_Voltage", 0)
        # SmartDashboard.putNumber("Test_ELEVATOR_Voltage", 0)
        # SmartDashboard.getNumber("ELEVATOR_P_VALUE", 0)
        # SmartDashboard.putNumber("ARM_Voltage", 0)

        OI.init()
        OI.map_controls()

        self.pv_selection = wpilib.SendableChooser()
        self.pv_selection.setDefaultOption("On", "on")
        self.pv_selection.addOption("Off", "off")
        wpilib.SmartDashboard.putData("Photonvision", self.pv_selection)

        self.auto_selection = wpilib.SendableChooser()

        self.auto_selection.setDefaultOption(
            "Blue Balance Auto", autonomous.BlueBalanceAuto
        )

        self.auto_selection.addOption(
            "Blue Two Piece No Guard", autonomous.TWO_PIECE_NO_GUARD_BLUE
        )
        self.auto_selection.addOption(
            "Red Two Piece No Guard", autonomous.TWO_PIECE_NO_GUARD_RED
        )
        self.auto_selection.addOption(
            "Blue Three Piece With Guard", autonomous.THREE_PIECE_WITH_GUARD_BLUE
        )
        self.auto_selection.addOption(
            "Red Three Piece With Guard", autonomous.THREE_PIECE_WITH_GUARD_RED
        )
        self.auto_selection.addOption(
            "Blue 2.5 Piece Balance With Guard",
            autonomous.TWO_PIECE_PICK_BALANCE_WITH_GUARD_BLUE,
        )
        self.auto_selection.addOption(
            "Red 2.5 Piece Balance With Guard",
            autonomous.TWO_PIECE_PICK_BALANCE_WITH_GUARD_RED,
        )
        self.auto_selection.addOption(
            "Blue 2 Piece Balance With Guard",
            autonomous.TWO_PIECE_BALANCE_WITH_GUARD_BLUE,
        )
        self.auto_selection.addOption(
            "Red 2 Piece Balance With Guard",
            autonomous.TWO_PIECE_BALANCE_WITH_GUARD_RED,
        )

        self.auto_selection.addOption(
            "Do Nothing", AutoRoutine(Pose2d(0, 0, 0), InstantCommand(lambda: None))
        )

        self.can_reset_climb = wpilib.SendableChooser()

        self.can_reset_climb.addOption("Yes", True)

        self.can_reset_climb.setDefaultOption("No", False)

        wpilib.SmartDashboard.putData("Auto Mode", self.auto_selection)
        wpilib.SmartDashboard.putData("Can Reset Climber", self.can_reset_climb)

    def robotPeriodic(self):
        SmartDashboard.putNumber("Climber Rotations", Robot.climber.get_motor())
        SmartDashboard.putBoolean("Climbed", Robot.climber.is_climbed())
        SmartDashboard.putNumber(
            "Robot Roll", math.degrees(Sensors.gyro.get_robot_roll())
        )
        SmartDashboard.putNumber(
            "Robot Pitch", math.degrees(Sensors.gyro.get_robot_pitch())
        )
        SmartDashboard.putNumber(
            "Robot Yaw", math.degrees(Sensors.gyro.get_robot_heading())
        )
        SmartDashboard.putBoolean(
            "Zero Elevator", Robot.arm.elevator_bottom_sensor.get()
        )
        # SmartDashboard.putNumber("PITCH", Robot.drivetrain.gyro.get_robot_pitch())
        # SmartDashboard.putNumber("ARM_REAL", math.degrees(Robot.arm.get_rotation()))
        #
        # Sensors.odometry.update()
        # SmartDashboard.putString("ODOM", str(Robot.drivetrain.odometry.getPose()))
        # SmartDashboard.putString("FDOM", str(Sensors.odometry.getPose()))
        # SmartDashboard.putNumber("Current_length_meters", Robot.arm.get_length())
        # SmartDashboard.putString(
        #     "EDOM", str(Robot.drivetrain.odometry_estimator.getEstimatedPosition())
        # )
        # try:
        #     SmartDashboard.putString(
        #         "PHOTON", str(Sensors.pv_controller.get_estimated_robot_pose())
        #     )
        #     SmartDashboard.putString(
        #         "PHOTON ANGLE",
        #         str(
        #             Sensors.pv_controller.get_estimated_robot_pose()[0][0]
        #             .rotation()
        #             .toRotation2d()
        #             .degrees()
        #         ),
        #     )
        # except Exception:
        #     pass
        #
        Sensors.odometry.update()
        pose = Robot.drivetrain.odometry_estimator.getEstimatedPosition()
        # pose2 = Sensors.odometry.getPose()
        #
        SmartDashboard.putNumberArray(
            "RobotPoseAdvantage", [pose.X(), pose.Y(), pose.rotation().radians()]
        )
        SmartDashboard.putNumber(
            "ELEVATOR CURRENT", Robot.arm.motor_extend.motor.getOutputCurrent()
        )
        #
        # SmartDashboard.putNumberArray(
        #     "RobotPoseOrig", [pose2.X(), pose2.Y(), pose2.rotation().radians()]
        # )
        #
        try:
            pv_pose = Sensors.pv_controller.get_estimated_robot_pose()
            SmartDashboard.putNumberArray(
                "PVPoseAdvantage",
                [
                    pv_pose[0][0].toPose2d().X(),
                    pv_pose[0][0].toPose2d().Y(),
                    pv_pose[0][0].rotation().toRotation2d().radians(),
                ],
            )
        except Exception:
            pass
        #
        # SmartDashboard.putNumber(
        #     "gyro_angle: ", math.degrees(Robot.drivetrain.gyro.get_robot_heading())
        # )
        #
        SmartDashboard.putNumber(
            "SHOULDER ANGLE: ", math.degrees(Robot.arm.get_rotation())
        )
        SmartDashboard.putNumber(
            "WRIST ANGLE: ", math.degrees(Robot.grabber.get_angle())
        )
        SmartDashboard.putNumber("SHOULDER DIST: ", Robot.arm.get_length())

        try:
            commands2.CommandScheduler.getInstance().run()
        except Exception as e:
            print(e)

    def teleopInit(self):
        Robot.climber.climber_disable()
        Robot.climber.climber_motor.set_sensor_position(0)

        logger.debug("TELEOP", "Teleop Initialized")

        config.blue_team = self.auto_selection.getSelected().blue_team

        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(Robot.drivetrain)
        )

        Robot.arm.arm_rotation_motor.pid_controller.setOutputRange(-0.2, 0.2, slotID=1)
        commands2.CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                command.ZeroElevator(Robot.arm),
                command.ZeroShoulder(Robot.arm),
                command.ZeroWrist(Robot.grabber),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    config.scoring_locations["standard"],
                ),
            )
        )

        if self.pv_selection.getSelected() == "on":
            Sensors.pv_controller = PV_Cameras(
                constants.ApriltagPositionDictBlue
                if config.blue_team
                else constants.ApriltagPositionDictRed
            )
            Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)
        else:
            Sensors.pv_controller = None
            Sensors.odometry = FieldOdometry(Robot.drivetrain, None)

    def teleopPeriodic(self):
        ...

    def autonomousInit(self):
        config.blue_team = self.auto_selection.getSelected().blue_team
        if self.pv_selection.getSelected() == "on":
            Sensors.pv_controller = PV_Cameras(
                constants.ApriltagPositionDictBlue
                if config.blue_team
                else constants.ApriltagPositionDictRed
            )
            Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)
        else:
            Sensors.pv_controller = None
            Sensors.odometry = FieldOdometry(Robot.drivetrain, None)
        Robot.arm.arm_rotation_motor.pid_controller.setOutputRange(-0.2, 0.2, slotID=1)
        self.auto_selection.getSelected().run()

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
