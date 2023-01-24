import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight
from oi.OI import OI
from wpilib import SmartDashboard
import command
import constants
from robot_systems import Robot

from robot_systems import Robot, Sensors
from sensors import FieldOdometry, PV_Cameras


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        Sensors.limelight_front = Limelight(cam_height=0, cam_angle=0, robot_ip="10.74.07.2")
        Sensors.limelight_controller = LimelightController([Sensors.limelight_front])

        Robot.drivetrain.init()

        SmartDashboard.init()
        Sensors.pv_controller = PV_Cameras()

        Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)

        # self.start_limelight_pose = Sensors.limelight_controller.get_estimated_robot_pose()[0].toPose2d()
        # self.start_robot_pose = Sensors.odometry.get_robot_pose()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        limelight_bot_pose = Sensors.limelight_controller.get_estimated_robot_pose()
        pv_bot_pose = Sensors.pv_controller.get_estimated_robot_pose()
        current_robot_pose = Sensors.odometry.update()

        # print("LIMELIGHT POSE: ", limelight_bot_pose)
        # print("PHOTON_VI POSE: ", pv_bot_pose)
        #
        # print("ODOMETRY| POSE: ", Sensors.odometry.update())
        #
        # SmartDashboard.putString("LIMELIGHT POSE", str(limelight_bot_pose))
        # SmartDashboard.putString("PHOTON_VI POSE", str(pv_bot_pose))
        SmartDashboard.putString("ODOMETRY POSE", str(current_robot_pose))
        SmartDashboard.putString("DRIVETRAIN POSE", str(Robot.drivetrain.odometry.getPose()))

        # SmartDashboard.putString("ROBOT_EST_POSE_X", str(round(current_robot_pose.x - self.start_robot_pose.x, 2)))
        # SmartDashboard.putString("ROBOT_EST_POSE_Y", str(round(current_robot_pose.y - self.start_robot_pose.y, 2)))

        # print(Sensors.limelight_controller.limelights[0].table.getValue("json", None))

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().schedule(
            command.DrivetrainZero(Robot.drivetrain)
        )
        pass

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
    # Robot.robotInit(Robot())
