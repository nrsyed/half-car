from car import Car
from plot_sim import PlotSim


def simulate(time_step=0.0002):
    car = Car()
    elapsed_time = 0
    plot = PlotSim(car)
    exit()

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
