from units.SI import meters, radians

base_initial_coords: (meters, meters, radians) = (1.5, 0.57, 0)

base_path_1: ((meters, meters, radians), (meters, meters, radians)) = (
    (base_initial_coords[0], base_initial_coords[1], base_initial_coords[2]),
    (6.55, 1, 0),
)

base_path_2: ((meters, meters, radians), (meters, meters, radians)) = (
    base_path_1[1],
    (1.8, 1.18, 0),
)
