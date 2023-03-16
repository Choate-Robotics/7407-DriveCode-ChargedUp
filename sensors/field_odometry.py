import math
import time

from robotpy_toolkit_7407.sensors.odometry import VisionEstimator
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d

from subsystem import Drivetrain
from units.SI import seconds


def weighted_pose_average(
    robot_pose: Pose2d, vision_pose: Pose3d, robot_weight: float, vision_weight: float
) -> Pose2d:
    """
    Returns a weighted average of two poses.
    :param robot_pose: Pose of the robot.
    :type robot_pose: Pose2d
    :param vision_pose: Pose of the limelight.
    :type vision_pose: Pose3d
    :param robot_weight: Weight of the robot pose.
    :type robot_weight: float
    :param vision_weight: Weight of the limelight pose.
    :type vision_weight: float
    :return: Weighted average of the two poses.
    :rtype: Pose2d
    """

    vision_pose = vision_pose.toPose2d()

    return Pose2d(
        Translation2d(
            (
                robot_pose.translation().X() * robot_weight
                + vision_pose.translation().X() * vision_weight
            ),
            (
                robot_pose.translation().Y() * robot_weight
                + vision_pose.translation().Y() * vision_weight
            ),
        ),
        Rotation2d(
            robot_pose.rotation().radians() * robot_weight
            + vision_pose.rotation().radians() * vision_weight
        ),
    )


class FieldOdometry:
    """
    Keeps track of robot position relative to field using a vision estimator (e.g. limelight, photon-vision)
    """

    def __init__(
        self, drivetrain: Drivetrain, vision_estimator: VisionEstimator | None
    ):
        self.drivetrain: Drivetrain = drivetrain

        self.last_update_time: seconds | None = None
        self.min_update_wait_time: seconds = (
            0.05  # time to wait before checking for pose update
        )

        self.vision_estimator: VisionEstimator = vision_estimator
        self.vision_estimator_pose_weight: float = 0.1
        self.robot_pose_weight: float = 1 - self.vision_estimator_pose_weight

        self.vision_on = True

    def update(self) -> Pose2d:
        """
        Updates the robot's pose relative to the field. This should be called periodically.
        """

        self.drivetrain.odometry.update(
            self.drivetrain.get_heading(), *self.drivetrain.node_positions
        )

        self.drivetrain.odometry_estimator.update(
            self.drivetrain.get_heading(),
            self.drivetrain.node_positions,
        )

        vision_robot_pose_list: list[Pose3d] | None = None

        if self.vision_on:
            current_time = time.time()
            if self.last_update_time is None or (
                current_time - self.last_update_time >= self.min_update_wait_time
            ):
                vision_robot_pose_list = (
                    self.vision_estimator.get_estimated_robot_pose()
                    if self.vision_estimator
                    else None
                )

            if vision_robot_pose_list:
                for vision_robot_pose in vision_robot_pose_list:
                    if vision_robot_pose[0] and vision_robot_pose[1]:
                        vision_time = vision_robot_pose[1]
                        vision_robot_pose = vision_robot_pose[0]

                        angle_diff = (
                            math.degrees(
                                vision_robot_pose.toPose2d().rotation().radians()
                                - self.drivetrain.gyro.get_robot_heading()
                            )
                            % 360
                        )
                        angle_diff_rev = 360 - angle_diff

                        if (5 > angle_diff > -5) or (5 > angle_diff_rev > -5):
                            self.drivetrain.odometry_estimator.addVisionMeasurement(
                                vision_robot_pose.toPose2d(), vision_time
                            )

                            weighted_pose = weighted_pose_average(
                                self.drivetrain.odometry.getPose(),
                                vision_robot_pose,
                                self.robot_pose_weight,
                                self.vision_estimator_pose_weight,
                            )

                            self.drivetrain.odometry.resetPosition(
                                self.drivetrain.get_heading(),
                                weighted_pose,
                                *self.drivetrain.node_positions
                            )

                        self.last_update_time = current_time

        return self.getPose()

    def getPose(self) -> Pose2d:
        """
        Returns the robot's pose relative to the field.
        :return: Robot pose.
        :rtype: Pose2d
        """
        # return self.drivetrain.odometry.getPose()
        return self.drivetrain.odometry_estimator.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d):
        """
        Resets the robot's odometry.

        :param pose: Pose of the robot to reset to.
        :type pose: Pose2d
        """
        self.drivetrain.reset_odometry(pose)
