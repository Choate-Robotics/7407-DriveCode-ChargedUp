"""
SI units for use in typing and conversion.
"""

# Distance
import math
from typing import Type

meters = float
meters_to_inches = 39.3701
meters_to_feet = meters_to_inches / 12
meters_to_yards = meters_to_feet / 3
meters_to_miles = meters_to_yards / 1760

inches = float
inches_to_meters = 1 / meters_to_inches
inches_to_feet = inches_to_meters * meters_to_feet
inches_to_yards = inches_to_meters * meters_to_yards
inches_to_miles = inches_to_meters * meters_to_miles

feet = float
feet_to_meters = 1 / meters_to_feet
feet_to_inches = feet_to_meters * meters_to_inches
feet_to_yards = feet_to_meters * meters_to_yards
feet_to_miles = feet_to_meters * meters_to_miles

yards = float
yards_to_meters = 1 / meters_to_yards
yards_to_inches = yards_to_meters * meters_to_inches
yards_to_feet = yards_to_meters * meters_to_feet
yards_to_miles = yards_to_meters * meters_to_miles

miles = float
miles_to_meters = 1 / meters_to_miles
miles_to_inches = miles_to_meters * meters_to_inches
miles_to_feet = miles_to_meters * meters_to_feet
miles_to_yards = miles_to_meters * meters_to_yards

# Rotation
rotations = float
rotations_to_degrees = 360
rotations_to_radians = 2 * math.pi

degrees = float
degrees_to_rotations = 1 / rotations_to_degrees
degrees_to_radians = degrees_to_rotations * rotations_to_radians

radians = float
radians_to_rotations = 1 / rotations_to_radians
radians_to_degrees = radians_to_rotations * rotations_to_degrees

# Time
seconds = float
seconds_to_minutes = 1 / 60
seconds_to_hours = seconds_to_minutes / 60
seconds_to_days = seconds_to_hours / 24

minutes = float
minutes_to_seconds = 1 / seconds_to_minutes
minutes_to_hours = minutes_to_seconds * seconds_to_hours
minutes_to_days = minutes_to_seconds * seconds_to_days

hours = float
hours_to_seconds = 1 / seconds_to_hours
hours_to_minutes = hours_to_seconds * seconds_to_minutes
hours_to_days = hours_to_seconds * seconds_to_days

days = float
days_to_seconds = 1 / seconds_to_days
days_to_minutes = days_to_seconds * seconds_to_minutes
days_to_hours = days_to_seconds * seconds_to_hours

# Velocity
meters_per_second: Type[float] = float
meters_per_second_squared: Type[float] = float

radians_per_second: Type[float] = float
radians_per_second__to__degrees_per_second: float = radians_to_degrees
radians_per_second__to__rotations_per_second: float = radians_to_rotations

degrees_per_second: Type[float] = float
degrees_per_second__to__radians_per_second: float = degrees_to_radians
degrees_per_second__to__rotations_per_second: float = degrees_to_rotations

rotations_per_second: Type[float] = float
rotations_per_second__to__radians_per_second: float = rotations_to_radians
rotations_per_second__to__degrees_per_second: int = rotations_to_degrees

rotations_per_minute: Type[float] = float
rotations_per_minute_per_meter: Type[float] = float

miles_per_hour: Type[float] = float
miles_per_hour_to_meters_per_second: float = miles_to_meters * hours_to_seconds

# rotation_motor_encoder_units = float
# rotation_motor_encoder_units__to__swerve_pod_rotations = 1 / constants.drivetrain_turn_gear_ratio
#
# swerve_pod_rotations = float
# swerve_pod_rotations__to__rotation_motor_encoder_units = 1 / rotation_motor_encoder_units__to__swerve_pod_rotations
#
# drive_motor_encoder_units = float
# drive_motor_encoder_units__to__meters = 1 / constants.drivetrain_move_gear_ratio
#
# meters__to__drive_motor_encoder_units = 1 / drive_motor_encoder_units__to__meters
