import math

from commands2 import SequentialCommandGroup, InstantCommand
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.custom_pathing import FollowPathCustom
from robot_systems import Robot

start_pose = Pose2d(0, 0, math.radians(0))
end_pose = Pose2d(2, 2, math.radians(0))
trajectory_config = TrajectoryConfig(10, 20)
trajectory_config.setStartVelocity(0)
trajectory_config.setEndVelocity(0)

trajectory = TrajectoryGenerator.generateTrajectory(
    start=start_pose,
    interiorWaypoints=[
        Translation2d(0.5, 0.5),
        Translation2d(1, 1),
        Translation2d(1.5, 1.5),
    ],
    end=end_pose,
    config=trajectory_config,
)

path = FollowPathCustom(
    Robot.drivetrain,
    trajectory,
    math.radians(0),
    constants.period,
)

auto = SequentialCommandGroup(
    InstantCommand(lambda: print(Robot.drivetrain.odometry.getPose())),
    path,
    InstantCommand(lambda: print(Robot.drivetrain.odometry.getPose())),
)

routine = AutoRoutine(start_pose, auto)
