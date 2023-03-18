from units.SI import meters, radians

base_initial_coords: (meters, meters, radians) = (1.5, 4.99, 0)

base_path_1: ((meters, meters, radians), (meters, meters, radians)) = (
    (base_initial_coords[0], base_initial_coords[1], base_initial_coords[2]),
    (6.55, 4.33, 0),
)

base_path_2: ((meters, meters, radians), (meters, meters, radians)) = (
    base_path_1[1],
    (1.8, 4.2, 0),
)
