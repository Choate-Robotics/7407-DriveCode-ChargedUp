import rev
from subsystem import Drivetrain, Claw
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig

MOVE_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
class Robot:
    drivetrain = Drivetrain()
    claw = Claw({
        "claw_motor": SparkMax(can_id=1, config=MOVE_CONFIG),
    })
    pass

class Pneumatics:
    pass

class Sensors:
    pass
