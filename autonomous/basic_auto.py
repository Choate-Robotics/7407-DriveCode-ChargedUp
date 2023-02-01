from wpimath.geometry import Pose2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig
from autonomous.custom_pathing import FollowPathCustom
from robot_systems import Robot
import constants
from commands2 import SequentialCommandGroup
from autonomous.auto_routine import AutoRoutine

import math

start_pose = Pose2d(0, 0, math.radians(0))
end_pose = Pose2d(1, 0, math.radians(0))
trajectory_config = TrajectoryConfig(10, 20)
trajectory_config.setStartVelocity(0)
trajectory_config.setEndVelocity(0)

trajectory = TrajectoryGenerator.generateTrajectory(
    start=start_pose, interiorWaypoints=[], end=end_pose, config=trajectory_config
)

path = FollowPathCustom(
    Robot.drivetrain,
    trajectory,
    math.radians(0),
    constants.period,
)

auto = SequentialCommandGroup(
    path,
)

routine = AutoRoutine(start_pose, auto)
