from halfcar import Car, PlotSim, Road


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
            car.set_accel(4.4)
        elif 8 <= elapsed_time < 11:
            car.set_accel(-9)
        elif 11 <= elapsed_time < 26:
            car.set_accel(4)
        elif 26 <= elapsed_time < 30:
            car.set_accel(0)
        elif 30 <= elapsed_time < 34:
            car.set_accel(-9)
        elif 34 <= elapsed_time < 38:
            car.set_accel(-4.5)
        elif 38 <= elapsed_time < 44:
            car.set_accel(-2)
        else:
            break
        ######################################################################
        ######################################################################

        car.update_state(time_step)
        elapsed_time += time_step
        iteration += 1

        # If animating the result, a low interval will result in potentially
        # extremely slow performance, as the animation figure will be updated
        # each time this function yields a value.
        if iteration % interval == 0:
            yield elapsed_time


if __name__ == "__main__":
    car = Car()
    # road = Road(length= 8, mode="triangle", frequency=1.8, amplitude= 0.1)
    # car = Car(road_func=road)

    time_step = 0.0005
    interval = 100
    generator = simulate(car, time_step=time_step, interval=interval)
    plot_sim = PlotSim(car, suspension=True)

    writer = "ffmpeg"
    fps = 1 / (time_step * interval)
    writer_args = ["-vcodec", "h264"]

    # Uncomment the arguments below to save a video file.
    plot_sim.animate(
        generator,
        #write_video=True, writer=writer, fps=fps,
        #writer_args=writer_args
    )
