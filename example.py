from halfcar import Car, PlotSim


def simulate(car, time_step=0.0002):
    """
    TODO
    """

    elapsed_time = 0
    while True:
        ######################################################################
        ######################################################################
        ########### SET THE DESIRED SIMULATION PARAMETERS HERE ###############
        if 0 <= elapsed_time < 2:
            car.set_accel(4)
        elif 2 <= elapsed_time < 4:
            car.set_accel(-5)
        elif 4 <= elapsed_time <= 10:
            car.set_accel(4)
        else:
            break
        ######################################################################
        ######################################################################

        car.update_state(time_step)
        elapsed_time += time_step

        yield elapsed_time


if __name__ == "__main__":
    car = Car()
    plot_sim = PlotSim(car, update_interval=100, suspension=True)
    sim_gen = simulate(car, time_step=0.0002)
    plot_sim.animate(sim_gen)
