import argparse
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
    argparser = argparse.ArgumentParser()
    argparser.add_argument("--mode", "-m", type=str, default="sine",
        help="Road profile mode: 'flat', 'sine' (default), 'square', "\
                "'triangle', 'bump'"
    )
    argparser.add_argument("--amplitude", "-a", type=float,
        help="Amplitude (in meters) for sine, square, triangle, and bump modes"
    )
    argparser.add_argument("--frequency", "-f", type=float,
        help="Frequency for sine, square, triangle, and bump modes"
    )
    argparser.add_argument("--time-step", "-t", type=float, default=0.0005,
        help="Simulation time step in seconds (default 0.0005)"
    )
    argparser.add_argument("--interval", "-i", type=int, default=100,
        help="Draw animation frame every <interval> time steps (default 100)"
    )
    argparser.add_argument("--write", "-w", action="store_true",
        help="Write resulting animation to a video file"
    )
    args = {
        arg: val for arg, val in vars(argparser.parse_args()).items()
        if val is not None
    }

    # Set road parameters based on car properties; note that the Car class
    # __init__() method automatically instantiates a `Road` object by default.
    # Below, we are overwriting this default.
    road_args = {
        "x_min": -3.3,
        "length": 6,
        "mode": args["mode"]
    }

    # Select reasonable default settings for the various modes.
    if args["mode"] == "square":
        road_args["amplitude"] = args.get("amplitude", 0.03)
        road_args["frequency"] = args.get("frequency", 0.1)
    elif args["mode"] == "sine":
        road_args["amplitude"] = args.get("amplitude", 0.3)
        road_args["frequency"] = args.get("frequency", 0.04)
    elif args["mode"] in ("triangle", "bump"):
        road_args["amplitude"] = args.get("amplitude", 0.05)
        road_args["frequency"] = args.get("frequency", 1.8)
    road = Road(**road_args)

    # Instantiate the `Car` object, passing in the `Road` object defined above.
    car = Car(road_func=road)

    # Create a `PlotSim` object, passing the `Car` object instantiated above.
    plot_sim = PlotSim(car, suspension=True)

    # Create a generator from the generator function `simulate()` defined
    # in this example script.
    generator = simulate(
        car, time_step=args["time_step"], interval=args["interval"]
    )

    # Settings for writing the animation to a video file.
    writer = "ffmpeg"
    fps = 1 / (args["time_step"] * args["interval"])
    writer_args = ["-vcodec", "h264"]

    # Call the `PlotSim` object's `animate()` method, passing in the required
    # generator (which updates the `Car` object's state each time it's called).
    # The video write parameters are optional; if the `--write`/`-w` option was
    # not supplied when calling this script, no video will be written and the
    # video write arguments will be ignored.
    plot_sim.animate(
        generator,
        write_video=args["write"], writer=writer, fps=fps, writer_args=writer_args
    )
