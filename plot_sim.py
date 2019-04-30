import math
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

class PlotSim:
    def __init__(self, car, suspension=False, update_interval=1):
        """
        TODO

        :param update_interval: How frequently to update the animation figure.
            If 1, every frame. If 2, every other frame, and so on.
        :type update_interval: int
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

        self.fig = fig
        self.ax = ax
        self.lines = lines
        self.car = car
        self.road_datum = road_datum
        self.wheel_datum = wheel_datum
        self.update_interval = update_interval
        self.iteration = 0

    @staticmethod
    def get_transformation_matrix(angle, x_offset, y_offset):
        return np.array([
            [math.cos(angle), -math.sin(angle), x_offset],
            [math.sin(angle), math.cos(angle), y_offset],
            [0, 0, 1]
        ])

    def update_animation(self, elapsed_time):
        """
        TODO
        """

        if self.iteration % self.update_interval != 0:
            self.iteration += 1
            return

        # Set line data based on car state.
        car = self.car
        wheel_datum = self.wheel_datum
        l_f, l_r = car.properties["l_f"], car.properties["l_r"]
        hub_radius = car.appearance["hub_radius"]
        y_c, phi, y_f, y_r = car.state["position"][:,0]

        transformation_matrix = self.get_transformation_matrix(phi, 0, y_c)
        chassis = car.appearance["chassis"]
        transformed_chassis = transformation_matrix @ chassis
        self.lines["chassis"].set_data(
            transformed_chassis[0,:], transformed_chassis[1,:]
        )

        self.lines["cog"].set_ydata(y_c)
        self.lines["front_tire"].set_ydata(
            car.appearance["wheel"][1,:] + wheel_datum + y_f
        )
        self.lines["front_hub"].set_ydata(
            car.appearance["hub"][1,:] + wheel_datum + y_f
        )
        self.lines["rear_tire"].set_ydata(
            car.appearance["wheel"][1,:] + wheel_datum + y_r
        )
        self.lines["rear_hub"].set_ydata(
            car.appearance["hub"][1,:] + wheel_datum + y_r
        )

        wheel_angle = (
            car.state["distance_traveled"] / car.appearance["wheel_radius"]
        )

        self.lines["front_marker"].set_data(
            [l_f, l_f + hub_radius * math.cos(-wheel_angle)],
            [wheel_datum + y_f, wheel_datum + y_f + hub_radius * math.sin(-wheel_angle)]
        )

        self.lines["rear_marker"].set_data(
            [-l_r, -l_r + hub_radius * math.cos(-wheel_angle)],
            [wheel_datum + y_r, wheel_datum + y_r + hub_radius * math.sin(-wheel_angle)]
        )

        self.lines["road"].set_ydata(car.road_profile[1] + self.road_datum)

        self.iteration += 1

    def animate(self, generator_func):
        anim = matplotlib.animation.FuncAnimation(
            self.fig, self.update_animation, frames=generator_func,
            interval=1, repeat=False
        )
        plt.show()