from unittest.mock import MagicMock
import pytest
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from subsystem.arm import Elevator


from logging import Logger

LOGGING = Logger("ARM_SYSTEMS_TESTS")

