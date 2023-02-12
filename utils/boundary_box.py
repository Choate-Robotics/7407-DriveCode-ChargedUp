import math

import constants


def boundary_box(self, angle: float) -> float:  # RE_RUNNING DOES NOT WORK
    """
    Returns a percentage of the max elevator height allowed at the given angle (limited by the robot extension limits)
    """
    if self.extension_override:
        # if the extension override is enabled, return 1, which will maximize the max elevator height
        return 1
    else:
        raw_angle = angle
        angle = abs(angle)

        def get_length_ratio(theta, adjacent):
            # Gets the length of the angle using cosine
            if theta == 0:
                v = adjacent - 17.5 / constants.max_elevator_height_delta
                return 1 if v > 1 else v

            max_length = (adjacent / math.cos(theta)) - 17.5
            # if the length of the angle is greater than the max_elevator_height, return one
            # else give a percentage of the height of the current line and the max_elevator_height
            # protects against any irregular numbers higher than the max elevator height by always
            # defaulting to 1
            if abs(max_length) > constants.max_elevator_height_delta:
                return 1
            else:
                res = abs(max_length / constants.max_elevator_height_delta)
                return res  # where is the hype at

        # Finds the length of each the dimensions for the boundary box using the constants
        top_vertical_boundary_from_pivot = (
                constants.vertical_boundary
                - constants.pivot_point_height
                - constants.top_boundary_buffer_gap
        )  # positive z
        bottom_vertical_boundary_from_pivot = (
                constants.pivot_point_height - constants.bottom_boundary_buffer_gap
        )  # negative z
        horizontal_boundary_length_from_pivot = (
                0.5 * (constants.robot_length)
                + constants.horizontal_boundary
                - constants.side_boundary_buffer_gap
        )  # positive x

        # the max angle of the boundary dimsensions
        # the top boundry max angle is the angle of the top boundry and horizontal boundry length from the pivot point
        top_boundary_max_angle = math.atan2(
            top_vertical_boundary_from_pivot, horizontal_boundary_length_from_pivot
        )
        # the bottom boundry max angle is the angle of the bottom and horizontal boundry length from the pivot point
        bottom_boundary_max_angle = math.atan2(
            bottom_vertical_boundary_from_pivot, horizontal_boundary_length_from_pivot
        ) + math.radians(90)
        # the side boundry angle can be inferred from the top and bottom boundry angles

        # since we can only extend past our bumper in one direction at a time, if the intakes are down, we limit the angle to the side with the intake
        if not self.intake_up:
            back_horizontal_boundary_length_from_pivot = (
                    -0.5 * (constants.robot_length) - constants.side_boundary_buffer_gap
            )
            back_horizontal_boundary_max_angle = math.atan2(
                top_vertical_boundary_from_pivot,
                back_horizontal_boundary_length_from_pivot,
            )
            if raw_angle < back_horizontal_boundary_max_angle:
                # if the angle is on the other side of the robot, return 0
                return 0

        # assuming the zero is vertical...
        # if the angle is less than the top boundry max angle, return the length ratio of the angle and the top boundry
        if angle < top_boundary_max_angle:
            return get_length_ratio(angle, top_vertical_boundary_from_pivot)
        # if the angle is between the top boundry max angle and the bottom boundry max angle, return the length ratio of the angle and the horizontal boundry
        elif top_boundary_max_angle < angle < bottom_boundary_max_angle:
            return get_length_ratio(
                math.radians(90) - angle, horizontal_boundary_length_from_pivot
            )
        # if the angle is greater than the bottom boundry max angle, return the length ratio of the angle and the bottom boundry
        elif angle > bottom_boundary_max_angle:
            return get_length_ratio(
                math.radians(180) - angle, bottom_vertical_boundary_from_pivot
            )
        else:
            return 1
