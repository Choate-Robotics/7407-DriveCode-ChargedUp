from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (1.55, 5.44, 0)

go_get_first_cube: path = (
    initial,
    [],
    (initial[0] + 5.25, initial[1] - 0.43, 0),
)

come_back_with_first_cube: path = (
    go_get_first_cube[2],
    [(initial[0] + 2.42, initial[1] - 0.25)],
    (initial[0] + 0.35, initial[1] - 0.65, 0),
)

go_get_second_cube: path = (
    come_back_with_first_cube[2],
    [
        (initial[0] + 1.85, initial[1] - 0.28),
        (initial[0] + 3.3, initial[1] - 0.23),
        (initial[0] + 4.1, initial[1] - 0.25),
    ],
    (initial[0] + 5.4, initial[1] - 1.83, -1),
)

come_back_with_second_cube: path = (
    go_get_second_cube[2],
    [
        (initial[0] + 4.1, initial[1] - 0.25),
        (initial[0] + 3.3, initial[1] - 0.23),
        (initial[0] + 1.85, initial[1] - 0.28),
    ],
    (initial[0] + 0.35, initial[1] - 0.7, 0),
)
