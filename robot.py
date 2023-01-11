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

        self.gyro = PigeonIMUGyro_Wrapper(13)
        self.camera = PhotonCamera("globalshuttercamera",
                                   Pose3d(Translation3d(0, 1, 2), Rotation3d(roll=1, pitch=2, yaw=3)),
                                   scale_constant=(1/1.1),
                                   height=1, pitch=1)

        # self.camera = PhotonCamera("C922_Pro_Stream_Webcam",
        #                            Pose3d(Translation3d(0, 0, 0), Rotation3d(roll=0, pitch=0, yaw=0)))

        self.field_layout = {
            'apriltags': {
                0: Pose3d(
                    Translation3d(x=0, y=0, z=0),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
                1: Pose3d(
                    Translation3d(x=5, y=4, z=2),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
                2: Pose3d(
                    Translation3d(x=2, y=3, z=6),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
                3: Pose3d(
                    Translation3d(x=10, y=3, z=1),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
                5: Pose3d(
                    Translation3d(x=10, y=3, z=1),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
                10: Pose3d(
                    Translation3d(x=10, y=3, z=1),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
                11: Pose3d(
                    Translation3d(x=10, y=3, z=1),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
                12: Pose3d(
                    Translation3d(x=10, y=3, z=1),
                    Rotation3d(roll=0, pitch=0, yaw=0)
                ),
            },
            'fieldLength': 50,
            'fieldWidth': 30
        }

        self.odometry = PhotonOdometry(
            self.camera,
            self.field_layout,
            self.gyro
        )

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

        self.odometry.refresh()

        if self.odometry.camera.hasTargets():
            print("Target Pose: ",
                  self.odometry.camera.latest_best_target.relative_pose.translation().toTranslation2d())
            print("Robot Pose: ", self.odometry.getRobotPose())
        else:
            print(self.odometry.camera.latest_best_target)

        if self.odometry.camera.latest_best_target is not None:
            print(self.odometry.camera.latest_best_target.relative_pose.x_feet, self.odometry.camera.latest_best_target.relative_pose.y_feet)

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
