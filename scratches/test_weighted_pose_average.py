from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d, Rotation3d, Translation3d

limelight_est = Pose3d(
    Translation3d(1, 1, 1),
    Rotation3d(1, 1, 1)
)

robot_pose = Pose2d(
    Translation2d(.5, .5),
    Rotation2d(.5)
)

weight_left = .9
weight_right = 1 - weight_left


def weighted_pose_average(pose1: Pose2d, pose2: Pose3d, weight1: float, weight2: float) -> Pose2d:
    """
    Returns a weighted average of two poses.
    :param pose1: First pose.
    :type pose1: Pose2d
    :param pose2: Second pose.
    :type pose2: Pose2d
    :param weight1: Weight of first pose.
    :type weight1: float
    :param weight2: Weight of second pose.
    :type weight2: float
    :return: Weighted average of two poses.
    :rtype: Pose2d
    """

    current_pose = pose1
    est_pose = pose2.toPose2d()

    return Pose2d(
        Translation2d(
            (current_pose.translation().X() * weight1 + est_pose.translation().X() * weight2),
            (current_pose.translation().Y() * weight1 + est_pose.translation().Y() * weight2)
        ),
        Rotation2d(
            (current_pose.rotation().radians() * weight1 + est_pose.rotation().radians() * weight2)
        )
    )


print(weighted_pose_average(robot_pose, limelight_est, weight_left, weight_right))
