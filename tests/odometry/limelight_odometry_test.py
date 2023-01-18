from unittest.mock import MagicMock
import pytest

from sensors import FieldOdometry

from logging import Logger

LOGGING = Logger("FIELD_ODOMETRY_TESTS")


@pytest.fixture
def field_odometry() -> FieldOdometry:
    """
    Create a FieldOdometry object with mocked dependencies.
    :return: A mocked FieldOdometry object.
    :rtype: FieldOdometry
    """
    mock_drivetrain: MagicMock = MagicMock()
    mock_drivetrain.odometry.getPose.return_value = [1, 1, 1]
    mock_drivetrain.gyro.get_robot_heading.return_value = 0
    mock_drivetrain.odometry_estimator.addVisionMeasurement.return_value = None
    mock_drivetrain.odometry.resetPosition.return_value = None

    limelight: MagicMock = MagicMock()
    limelight.get_bot_pose.return_value = [1, 1, 1, .5, .5, .5]

    field_odometry: MagicMock = MagicMock()
    field_odometry.drivetrain = mock_drivetrain
    field_odometry.robot_pose = [1, 1, 1]
    field_odometry.robot_pose_weight = .9
    field_odometry.limelight_pose_weight = 1 - field_odometry.robot_pose_weight
    field_odometry.min_update_wait_time = .01
    field_odometry.last_update_time = None

    return field_odometry


def test_field_odometry_init(field_odometry: FieldOdometry):
    assert field_odometry.drivetrain is not None


def test_field_odometry_get_vision_controller_pose(field_odometry: FieldOdometry):
    assert field_odometry.get_vision_controller_pose() is not None


def test_field_odometry_update(field_odometry: FieldOdometry):
    field_odometry.update()
    LOGGING.warning(f"\n{field_odometry.robot_pose}")
    assert field_odometry.robot_pose is not None


def test_field_odometry_get_robot_pose(field_odometry: FieldOdometry):
    LOGGING.warning(f"\n{field_odometry.get_robot_pose()}")
    assert field_odometry.get_robot_pose() is not None
