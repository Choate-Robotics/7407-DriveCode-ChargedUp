from unittest.mock import MagicMock
import pytest
from wpimath.geometry import Pose2d, Translation2d, Rotation2d

from sensors import FieldOdometry
from subsystem.drivetrain import Drivetrain
from robotpy_toolkit_7407.sensors.limelight import LimelightController, Limelight

from logging import Logger

LOGGING = Logger("FIELD_ODOMETRY_TESTS")


@pytest.fixture
def field_odometry() -> FieldOdometry:
    """
    Create a FieldOdometry object with mocked dependencies.
    :return: A mocked FieldOdometry object.
    :rtype: FieldOdometry
    """
    mock_drivetrain: Drivetrain = MagicMock()
    mock_drivetrain.odometry.getPose.return_value = Pose2d(
        Translation2d(0, 0),
        Rotation2d(1)
    )
    mock_drivetrain.gyro.get_robot_heading.return_value = 0
    mock_drivetrain.odometry_estimator.addVisionMeasurement.return_value = None
    mock_drivetrain.odometry.resetPosition.return_value = None

    mock_limelight: Limelight = MagicMock()
    mock_limelight.get_bot_pose.return_value = [1, 1, 1, 0, 0, 0]

    limelight_controller: LimelightController = LimelightController([mock_limelight])

    field_odometry: FieldOdometry = FieldOdometry(mock_drivetrain, limelight_controller)

    return field_odometry


def test_field_odometry_init(field_odometry: FieldOdometry):
    assert field_odometry.drivetrain is not None


def test_field_odometry_get_vision_controller_pose(field_odometry: FieldOdometry):
    assert field_odometry.vision_estimator.get_estimated_robot_pose() is not None


def test_field_odometry_update(field_odometry: FieldOdometry):
    field_odometry.update()
    LOGGING.warning(f"\n{field_odometry.robot_pose}")
    assert field_odometry.robot_pose is not None


def test_field_odometry_get_robot_pose(field_odometry: FieldOdometry):
    LOGGING.warning(f"\n{field_odometry.get_robot_pose()}")
    assert field_odometry.get_robot_pose() is not None
