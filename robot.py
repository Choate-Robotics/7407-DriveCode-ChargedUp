import commands2
import wpilib
from photonvision import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout, AprilTag
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper

from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Transform3d
from oi.OI import OI

from robotpy_toolkit_7407.sensors.photonvision import PhotonCamera, PhotonOdometry, PhotonTarget


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

        self.cam = PhotonCamera("hello", Pose3d(Translation3d(0, 1, 2), Rotation3d(roll=1, pitch=2, yaw=3)), height=1,
                                pitch=1)

        april_tag_1 = AprilTag()
        april_tag_1.ID = 1
        april_tag_1.pose = Pose3d(Translation3d(0, 0, 0), Rotation3d(roll=0, pitch=0, yaw=0))

        target = PhotonTarget(PhotonTrackedTarget(1, 1, 1, 1, 1, Transform3d(Pose3d(1, 1, 1, Rotation3d(1, 1, 1)), Pose3d(1, 1, 1, Rotation3d(1, 1, 1))), Transform3d(Pose3d(1, 1, 1, Rotation3d(1, 1, 1)), Pose3d(1, 1, 1, Rotation3d(1, 1, 1))), .1, [(1,1),(1,1),(1,1),(1,1)]))

        self.odometry = PhotonOdometry(
            self.cam,
            AprilTagFieldLayout(
                apriltags=[
                    april_tag_1
                ],
                fieldLength=50,
                fieldWidth=30
            ),
            self.gyro
        )

        self.odometry.refresh()
        print(self.odometry.getRobotPose())

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        self.cam.refresh()
        print(self.cam.latest_target)

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
    # wpilib.run(Robot)
    Robot.robotInit(Robot())