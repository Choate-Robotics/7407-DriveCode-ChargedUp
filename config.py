from wpimath.geometry import Pose2d

from command.targeting import TargetData

red_team: bool = True

elevator_motor_extend_id = 17
elevator_secondary_rotation_motor_id = 1
elevator_main_rotation_motor_id = 2
elevator_brake_id = 3
# converted to radians in subsystems/elevator.py

pneumatics_control_module = 31

claw_motor_speed: float = 0.4

# Intake
intake_motor_id = 11
intake_piston_forwardChannel = 5
intake_piston_reverseChannel = 4

default_intake_speed = 0.5

kRobotVisionPoseWeight = 0.1
# Dummy data
claw_motor_extend_id = 0

# SCORING LOCATIONS
scoring_locations = {
    "low": TargetData(
        target_pose=Pose2d(0, 0, 0),
        arm_angle=0,
        arm_length=0,
        wrist_angle=0,
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
    ),
    "middle": TargetData(
        target_pose=Pose2d(0, 0, 0),
        arm_angle=-44.78,
        arm_length=0.55,
        wrist_angle=-27.09,
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
    ),
    "high": TargetData(
        target_pose=Pose2d(0, 0, 0),
        arm_angle=-47.7,
        arm_length=1.04,
        wrist_angle=-18.61,
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
    ),
    "picking": TargetData(
        target_pose=None,
        arm_angle=-100,
        arm_length=0.099,
        wrist_angle=-20.53,
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=True,
    ),
    "standard": TargetData(
        target_pose=None,
        arm_angle=0,
        arm_length=0,
        wrist_angle=0,
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
    ),
}
