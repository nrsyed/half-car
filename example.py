from halfcar import Car, PlotSim


def simulate(car, time_step=0.0002, interval=1):
    """
    TODO
    """

    elapsed_time = 0
    iteration = 0
    while True:
        ######################################################################
        ######################################################################
        ########### SET THE DESIRED SIMULATION PARAMETERS HERE ###############
        if 0 <= elapsed_time < 8:
            car.set_accel(4)
        elif 8 <= elapsed_time < 14:
            car.set_accel(-9)
        elif 14 <= elapsed_time <= 16:
            car.set_accel(0)
        else:
            break
        ######################################################################
        ######################################################################

        car.update_state(time_step)
        elapsed_time += time_step
        iteration += 1

        if iteration % interval == 0:
            yield elapsed_time


if __name__ == "__main__":
    car = Car()
    sim_gen = simulate(car, time_step=0.0005, interval=100)
    plot_sim = PlotSim(car, suspension=True)
    plot_sim.animate(sim_gen)
