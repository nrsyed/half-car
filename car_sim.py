rom scipy import interpolate
from car import Car
from road import Road


def simulate(time_step=0.0002):
    car = Car()

    # The car COG is centered at x = 0. The road must extend left past the
    # rear wheel point of contact (x = -l_r) and right past the front
    # wheel point of contact (x = l_f). Choose road limits (x_min, x_max)
    # accordingly.
    road_limits = (-2 * car.properties["l_r"], 2 * car.properties["l_f"])
    road = Road(length=road_limits[1] - road_limits[0])

    elapsed_time = 0

    # Initial velocity and acceleration.

    while True:
        ######################################################################
        ######################################################################
        # SET THE DESIRED SIMULATION PARAMETERS HERE.

        if elapsed
