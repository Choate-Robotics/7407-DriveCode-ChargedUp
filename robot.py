import math

import commands2
import wpilib
from photonvision import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout, AprilTag
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper

from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Transform3d
from oi.OI import OI

from robotpy_toolkit_7407.sensors.photonvision import PhotonOdometry, PhotonTarget, PhotonCamera


class Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        self.gyro = PigeonIMUGyro_Wrapper(10)
        self.camera = PhotonCamera("Global_Shutter_Camera",
                                   Pose3d(Translation3d(x=.71/2, y=-.69/2, z=.65),
                                          Rotation3d(roll=0, pitch=0, yaw=0)),
                                   scale_constant=1)

        # self.camera = PhotonCamera("C922_Pro_Stream_Webcam",
        #                            Pose3d(Translation3d(0, 0, 0), Rotation3d(roll=0, pitch=0, yaw=0)))

        self.field_layout = {
            'apriltags': {
                1: Pose3d(
                    Translation3d(x=2, y=-2, z=0.70485),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
            },
            'fieldLength': 50,
            'fieldWidth': 30
        }

        self.odometry = PhotonOdometry(
            self.camera,
            self.field_layout,
            self.gyro,
            start_pose=Pose3d(Translation3d(0, 0, 0), Rotation3d(roll=0, pitch=0, yaw=0))
        )

        self.gyro.reset_angle()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

        self.odometry.refresh()

        print("Robot Pose: ", self.odometry.pose_estimate)
        print("Gyro Angle: ", math.degrees(self.gyro.get_robot_heading()))

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
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
