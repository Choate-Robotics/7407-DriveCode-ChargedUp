from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

base_initial_coords: coord = (1.5, 0.57, 0)

base_path_1: path = (
    base_initial_coords,
    [],
    (6.75, 1, 0),
)

base_path_2: path = (
    base_path_1[2],
    [(3.92, 0.82)],
    (1.6, 1.12, 0),
)

base_path_3: path = (
    base_path_2[2],
    [(3.35, 0.85), (4.8, 0.8), (5.6, 0.82)],
    (6.9, 2.4, 1),
)

base_path_4: path = (
    base_path_3[2],
    [(5.6, 0.85), (4.8, 0.8), (3.92, 0.85)],
    (1.6, 1.08, 0),
)
