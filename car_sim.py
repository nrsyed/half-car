from car import Car
from road import Road


def simulate(time_step=0.0002):
    car = Car()

    # The car COG is centered at x = 0. The road must extend left past the
    # rear wheel point of contact (x = -l_r) and right past the front
    # wheel point of contact (x = l_f). Choose road limits (x_min, x_max)
    # accordingly.
    road_limits = (-2 * car.properties["l_r"], 2 * car.properties["l_f"])
    road_length = road_limits[1] - road_limits[0]
    road = Road(length=road_length, x_min=road_limits[0])
    car.set_road_func(road)

    elapsed_time = 0

    # Initial horizontal velocity and acceleration.

    while True:
        ######################################################################
        ######################################################################
        ########### SET THE DESIRED SIMULATION PARAMETERS HERE ###############
        if 0 <= elapsed_time < 2:
            car.set_accel(4)
        elif 2 <= elapsed_time < 4:
            car.set_accel(-5)
        elif 4 <= elapsed_time <= 5:
            car.set_accel(4)
        else:
            break
        ######################################################################
        ######################################################################

        car.update_state(time_step)
        elapsed_time += time_step

if __name__ == "__main__":
    simulate()
