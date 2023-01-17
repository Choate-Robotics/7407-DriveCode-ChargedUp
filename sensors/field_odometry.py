import time

from subsystem import Drivetrain
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d, Rotation3d, Translation3d
from robotpy_toolkit_7407.sensors.limelight import Limelight


class FieldOdometry:
    def __init__(self, drivetrain: Drivetrain):
        self.drivetrain = drivetrain
        self.robot_pose: Pose2d | None = None

        self.limelight_front = Limelight(0, 0, robot_ip="10.74.07.2")
        self.limelight_robot_pose: Pose3d | None = None

        self.last_update_time = None
        self.min_update_wait_time = .05  # seconds to wait before checking for pose update

        self.limelight_pose_weight = .1
        self.robot_pose_weight = 1 - self.limelight_pose_weight

    def get_limelight_robot_pose(self) -> Pose3d | None:
        """
        Returns the robot's pose relative to the field, estimated by the limelight.
        :return: Limelight estimate of robot pose.
        :rtype: Pose3d
        """
        est_pose = self.limelight_front.get_bot_pose()
        if est_pose is None:
            return None

        return Pose3d(
            Translation3d(est_pose[0], est_pose[1], est_pose[2]),
            Rotation3d(est_pose[3], est_pose[4], est_pose[5])
        )

    def weighted_pose_average(self, robot_pose: Pose2d, limelight_pose: Pose3d, robot_weight: float,
                              limelight_weight: float) -> Pose2d:
        """
        Returns a weighted average of two poses.
        :param robot_pose: Pose of the robot.
        :type robot_pose: Pose2d
        :param limelight_pose: Pose of the limelight.
        :type limelight_pose: Pose3d
        :param robot_weight: Weight of the robot pose.
        :type robot_weight: float
        :param limelight_weight: Weight of the limelight pose.
        :type limelight_weight: float
        :return: Weighted average of the two poses.
        :rtype: Pose2d
        """

        limelight_pose = limelight_pose.toPose2d()

        return Pose2d(
            Translation2d(
                (robot_pose.translation().X() * robot_weight + limelight_pose.translation().X() * limelight_weight),
                (robot_pose.translation().Y() * robot_weight + limelight_pose.translation().Y() * limelight_weight)
            ),
            Rotation2d(
                robot_pose.rotation().radians() * robot_weight + limelight_pose.rotation().radians() * limelight_weight)
        )

    def update(self):
        self.robot_pose = Pose2d(
            self.drivetrain.odometry.getPose().translation(),
            Rotation2d(self.drivetrain.gyro.get_robot_heading())
        )

        current_time = time.time()
        if self.last_update_time is None or (current_time - self.last_update_time >= self.min_update_wait_time):
            self.limelight_robot_pose = self.get_limelight_robot_pose()

        if self.limelight_robot_pose is not None:
            weighted_pose = self.weighted_pose_average(
                self.robot_pose,
                self.limelight_robot_pose,
                self.robot_pose_weight,
                self.limelight_pose_weight
            )

            self.drivetrain.odometry.resetPosition(
                weighted_pose,
                self.robot_pose.rotation(),
            )

            self.robot_pose = Pose2d(
                self.drivetrain.odometry.getPose().translation(),
                Rotation2d(self.drivetrain.gyro.get_robot_heading())
            )

            self.last_update_time = current_time
