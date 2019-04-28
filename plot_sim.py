import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PlotSim:
    def __init__(self, car, suspension=False):
        """
        TODO
        """

        fig, ax = plt.subplots()

        chassis = car.appearance["chassis"]
        wheel = car.appearance["wheel"]
        wheel_radius = car.appearance["wheel_radius"]
        hub = car.appearance["hub"]
        hub_radius = car.appearance["hub_radius"]
        lowest_point = car.appearance["lowest_point"]
        ground_clearance = car.appearance["ground_clearance"]

        l_f = car.properties["l_f"]
        l_r = car.properties["l_r"]

        # The half-car suspension model does not take absolute positions of
        # components into account, only their displacements from their initial
        # positions. e.g., the physical difference in height between the car
        # COG and the wheel centers or between the wheel centers and the ground
        # are not part of the model state. Thus, we calculate these differences
        # here explicitly to correctly display the vehicle.
        wheel_datum = lowest_point - ground_clearance + wheel_radius

        # Set vehicle component colors.
        chassis_color = "k"
        front_wheel_color = "g"
        rear_wheel_color="b"

        # Initialize plot axis line objects and store in dict.
        lines = {}

        # Chassis lines.
        lines["chassis"], = ax.plot(
            chassis[0,:], chassis[1,:], c=chassis_color, lw=2
        )
        
        lines["cog"], = ax.plot(0, 0, c=chassis_color, marker="o")

        # Front wheel lines.
        lines["front_tire"], = ax.plot(
            wheel[0,:] + l_f, wheel[1,:] + wheel_datum, c=front_wheel_color,
            lw=2, zorder=2
        )

        lines["front_hub"], = ax.plot(
            hub[0,:] + l_f, hub[1,:] + wheel_datum, c=front_wheel_color,
            lw=2, zorder=2
        )

        lines["front_marker"], = ax.plot(
            [l_f, l_f + hub_radius], [wheel_datum, wheel_datum],
            c=front_wheel_color, lw=1
        )

        # Rear wheel lines.
        lines["rear_tire"], = ax.plot(
            wheel[0,:] - l_r, wheel[1,:] + wheel_datum, c=rear_wheel_color, lw=2
        )

        lines["rear_hub"], = ax.plot(
            hub[0,:] - l_r, hub[1,:] + wheel_datum, c=rear_wheel_color, lw=2
        )

        lines["rear_marker"], = ax.plot(
            [-l_r, -l_r + hub_radius], [wheel_datum, wheel_datum],
            c=rear_wheel_color, lw=1
        )

        # Suspension.
        # TODO

        # Set axis limits and road lines.
        road_profile = car.road_profile
        x_limits = (car.road_profile[0][0], car.road_profile[0][-1])
        y_limits = (-2, 2)
        ax.set_ylim(y_limits)
        ax.set_xlim(x_limits)

        road_color = "brown"

        road_datum = lowest_point - ground_clearance
        lines["road"], = ax.plot(
            road_profile[0], road_profile[1] + road_datum, c=road_color, lw=1.5
        )

        # lines["road_marker"], = TODO

        plt.show()
