import config
from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_base_initial_coords: coord = (1.5, 0.57, 0)

blue_base_path_1: path = (
    blue_base_initial_coords,
    [],
    (6.55, config.scoring_width - 1, 0),
)

red_base_initial_coords: coord = (1.5, config.field_width - 0.57, 0)

red_base_path_1: path = (
    red_base_initial_coords,
    [],
    (6.6, config.field_width - (config.scoring_width - 1), 0),
)
