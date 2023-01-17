import math
import constants
import ctre
import rev
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveDrivetrain

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


class SparkMaxSwerveNode(SwerveNode):
    m_move: SparkMax
    m_turn: SparkMax
    encoder: ctre.CANCoder
    absolute_encoder_zeroed_pos: float = 0  # Radians
    drive_reversed: bool = False
    turn_reversed: bool = False

    def init(self):
        super().init()
        self.m_turn.init()
        self.m_move.init()


class Drivetrain(SwerveDrivetrain):
    pass
