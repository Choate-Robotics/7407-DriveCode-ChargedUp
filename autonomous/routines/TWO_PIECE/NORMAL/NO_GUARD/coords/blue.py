from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (1.5, 0.57, 0)

go_get_first_cube: path = (
    initial,
    [],
    (initial[0] + 5.25, initial[1] + .43, 0),
)

come_back_with_first_cube: path = (
    go_get_first_cube[2],
    [
        (initial[0] + 2.42, initial[1] + .25)
    ],
    (initial[0] + .1, initial[1] + .65, 0),
)
