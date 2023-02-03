from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig


class CustomTrajectory:
    """
    Wrapper for the wpimath trajectory class with additional info.

    :param start_pose: Starting pose.
    :type start_pose: Pose2d
    :param waypoints: Waypoints.
    :type waypoints: list[Translation2d]
    :param end_pose: Ending pose.
    :type end_pose: Pose2d
    :param max_velocity: Maximum velocity.
    :type max_velocity: float (meters per second)
    :param max_accel: Maximum acceleration.
    :type max_accel: float (meters per second squared)
    :param start_velocity: Starting velocity.
    :type start_velocity: float (meters per second)
    :param end_velocity: Ending velocity.
    :type end_velocity: float (meters per second)
    """

    def __init__(self,
                 start_pose: Pose2d,
                 waypoints: list[Translation2d],
                 end_pose: Pose2d,
                 max_velocity: float,
                 max_accel: float,
                 start_velocity: float = 0,
                 end_velocity: float = 0,
                 ):
        self.start_pose = start_pose
        self.waypoints = waypoints
        self.end_pose = end_pose
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.start_velocity = start_velocity
        self.end_velocity = end_velocity

        config = TrajectoryConfig(
            self.max_velocity,
            self.max_accel,
        )
        config.setStartVelocity(self.start_velocity)
        config.setEndVelocity(self.end_velocity)

        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=self.start_pose,
            interiorWaypoints=self.waypoints,
            end=self.end_pose,
            config=config
        )
