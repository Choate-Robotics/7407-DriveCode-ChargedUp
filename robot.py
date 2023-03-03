import math

import commands2
import wpilib
from commands2 import InstantCommand, SequentialCommandGroup
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

import autonomous
import command
import config
from autonomous.auto_routine import AutoRoutine
from oi.OI import OI
from robot_systems import Pneumatics, Robot, Sensors
from sensors import FieldOdometry, PV_Cameras
from utils import logger


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.team_selection: wpilib.SendableChooser | None = None
        self.auto_selection: wpilib.SendableChooser | None = None

    def robotInit(self):
        period = 0.03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        Pneumatics.compressor.enableAnalog(90, 120)
        Robot.arm.init()
        Robot.drivetrain.init()
        Robot.intake.init()
        Robot.grabber.init()

        Sensors.pv_controller = None
        Sensors.odometry = FieldOdometry(Robot.drivetrain, None)
        Sensors.gyro = Robot.drivetrain.gyro

        # SmartDashboard.putNumber("ELEVATOR_Voltage", 0)
        # SmartDashboard.putNumber("Test_ELEVATOR_Voltage", 0)
        # SmartDashboard.getNumber("ELEVATOR_P_VALUE", 0)
        # SmartDashboard.putNumber("ARM_Voltage", 0)

        OI.init()
        OI.map_controls()

        self.team_selection = wpilib.SendableChooser()
        self.team_selection.setDefaultOption("Blue", "blue")
        self.team_selection.addOption("Red", "red")
        wpilib.SmartDashboard.putData("Team Selection", self.team_selection)

        self.auto_selection = wpilib.SendableChooser()
        self.auto_selection.setDefaultOption(
            "Blue Basic Auto", autonomous.BlueBasicAuto
        )
        self.auto_selection.addOption("Blue Balance Auto", autonomous.BlueBalanceAuto)
        self.auto_selection.addOption(
            "Blue Cone Cube Right", autonomous.BlueConeCubeScoreRight
        )
        self.auto_selection.addOption(
            "Blue Cone Cube Left", autonomous.BlueConeCubeScoreLeft
        )
        self.auto_selection.addOption("Red Basic Auto", autonomous.RedBasicAuto)
        self.auto_selection.addOption("Red Balance Auto", autonomous.RedBalanceAuto)
        self.auto_selection.addOption(
            "Red Cone Cube Right", autonomous.RedConeCubeScoreRight
        )
        self.auto_selection.addOption(
            "Red Cone Cube Left", autonomous.RedConeCubeScoreLeft
        )
        self.auto_selection.addOption("Square Auto", autonomous.SquareAuto)
        self.auto_selection.addOption(
            "Do Nothing", AutoRoutine(Pose2d(0, 0, 0), InstantCommand(lambda: None))
        )

        wpilib.SmartDashboard.putData("Auto Mode", self.auto_selection)

    def robotPeriodic(self):
        SmartDashboard.putBoolean("Team", config.red_team)
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
        except Exception:
            ...

    def teleopInit(self):
        logger.debug("TELEOP", "Teleop Initialized")

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

    def teleopPeriodic(self):
        config.red_team = False
        # SmartDashboard.putBoolean(
        #     "Limit Switch", Robot.arm.elevator_bottom_sensor.get()
        # )
        # print("Limit Switch: ", Robot.arm.elevator_bottom_sensor.get())

        # SmartDashboard.putBoolean("ELEVATOR_BOTTOM_SENSOR", Robot.arm.elevator_bottom_sensor.get())
        # print(Pneumatics.compressor.getPressure())
        # print("I THINK I'm AT: ", math.degrees(Robot.Arm.get_rotation()))
        # Robot.Arm.disable_brake()
        ...

    def autonomousInit(self):
        config.red_team = False
        if self.team_selection.getSelected() == "blue":
            Sensors.pv_controller = PV_Cameras()
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
