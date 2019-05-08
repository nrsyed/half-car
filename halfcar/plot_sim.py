from datetime import datetime
import math
import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import scipy
from scipy import interpolate

from .shapeutil import zigzag


class PlotSim:
    def __init__(
        self, car, suspension=False, road_marker_interval=0,
        road_marker_vertical_offset=0.25
    ):
        """
        TODO

        :param road_marker_interval: distance between road markers in meters.
            Must be larger than the x span of the road profile (since only
            one road marker can currently be displayed on the plot at a time).
        :type road_marker_interval: int
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

        # Chassis and suspension lines.
        lines["chassis"], = ax.plot(
            chassis[0,:], chassis[1,:], c=chassis_color, lw=2
        )
        
        lines["cog"], = ax.plot(0, 0, c=chassis_color, marker="o")

        lines["front_suspension"], = ax.plot(
            [], [], c=chassis_color, lw=1, zorder=1
        )

        lines["rear_suspension"], = ax.plot(
            [], [], c=chassis_color, lw=1, zorder=1
        )

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

        # Set axis limits and road lines.
        road_profile = car.road_profile
        x_limits = (car.road_profile[0][0], car.road_profile[0][-1])
        y_limits = (-2, 2)
        ax.set_aspect("equal")
        ax.set_ylim(y_limits)
        ax.set_xlim(x_limits)

        road_color = "brown"

        road_datum = lowest_point - ground_clearance
        lines["road"], = ax.plot(
            road_profile[0], road_profile[1] + road_datum, c=road_color, lw=1.5
        )

        # Ensure `road_marker_interval` is an int greater than the x span of the plot.
        road_marker_interval = max(int(road_marker_interval), math.ceil(x_limits[1] - x_limits[0]))
        lines["road_marker"], = ax.plot(
            road_profile[0][-1], road_datum - road_marker_vertical_offset,
            c=road_color, marker="^"
        )

        # Information display text annotations.
        annotations = {}
        annotations["time"] = ax.annotate(
            "", xy=(0.01, 0.02), xycoords="axes fraction"
        )
        annotations["brake_max_speed"] = ax.annotate(
            "", xy=(0.2, 0.18), xycoords="axes fraction", color="r"
        )
        annotations["accel"] = ax.annotate(
            "", xy=(0.2, 0.14), xycoords="axes fraction"
        )
        annotations["speed"] = ax.annotate(
            "", xy=(0.2, 0.1), xycoords="axes fraction"
        )
        annotations["speed_kph"] = ax.annotate(
            "", xy=(0.267, 0.06), xycoords="axes fraction"
        )
        annotations["speed_mph"] = ax.annotate(
            "", xy=(0.267, 0.02), xycoords="axes fraction"
        )
        annotations["dist"] = ax.annotate(
            "", xy=(0.5, 0.1), xycoords="axes fraction"
        )
        annotations["dist_ft"] = ax.annotate(
            "", xy=(0.664, 0.06), xycoords="axes fraction"
        )
        annotations["dist_mi"] = ax.annotate(
            "", xy=(0.664, 0.02), xycoords="axes fraction"
        )

        self.suspension = suspension
        self.fig = fig
        self.ax = ax
        self.annotations = annotations
        self.lines = lines
        self.car = car
        self.road_datum = road_datum
        self.road_marker_interval = road_marker_interval
        self.road_marker_vertical_offset = road_marker_vertical_offset
        self.wheel_datum = wheel_datum
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

        road_x_limits = (car.road_profile[0][0], car.road_profile[0][-1])
        road_marker_x = (
            road_x_limits[1]
            - (car.state["distance_traveled"] % self.road_marker_interval)
        )
        
        # Use an interpolation to get the height of the road at the road
        # marker x coordinate, then set the y coordinate of the road
        # marker accordingly.
        road_interpolation = scipy.interpolate.interp1d(
            car.road_profile[0], car.road_profile[1], bounds_error=False
        )
        road_marker_y = (
            self.road_datum
            + road_interpolation(road_marker_x)
            - self.road_marker_vertical_offset
        )
        self.lines["road_marker"].set_data(road_marker_x, road_marker_y)

        if self.suspension:
            front_well_top = car.appearance["front_well_top"]
            rear_well_top = car.appearance["rear_well_top"]
            transformed_front_well_top = transformation_matrix @ front_well_top
            transformed_rear_well_top = transformation_matrix @ rear_well_top

            num_spring_nodes = 10
            spring_width = 0.2
            front_spring = zigzag(
                transformed_front_well_top[:2,:],
                np.array([[l_f], [wheel_datum + y_f]]),
                num_spring_nodes,
                spring_width
            )

            rear_spring = zigzag(
                transformed_rear_well_top[:2,:],
                np.array([[-l_r], [wheel_datum + y_r]]),
                num_spring_nodes,
                spring_width
            )

            self.lines["front_suspension"].set_data(front_spring)
            self.lines["rear_suspension"].set_data(rear_spring)

        # Update annotations.
        self.annotations["time"].set_text("T = {:.2f} s".format(elapsed_time))
        self.annotations["accel"].set_text(
            "a =  {:.2f} m/s^2".format(car.state["horizontal_accel"])
        )
        self.annotations["speed"].set_text(
            "v =  {:.2f} m/s".format(car.state["horizontal_velocity"])
        )
        self.annotations["speed_kph"].set_text(
            "{:.1f} kph".format(car.state["horizontal_velocity"] * 3.6)
        )
        self.annotations["speed_mph"].set_text(
            "{:.1f} mph".format(car.state["horizontal_velocity"] * 3.6 * 0.621)
        )
        self.annotations["dist"].set_text(
            "Distance = {:.1f} m".format(car.state["distance_traveled"])
        )
        self.annotations["dist_ft"].set_text(
            "{:.1f} ft".format(car.state["distance_traveled"] * 3.28)
        )
        self.annotations["dist_mi"].set_text(
            "{:.2f} mi".format(car.state["distance_traveled"] * 3.28 / 5280)
        )

        if car.state["horizontal_accel"] < 0:
            self.annotations["brake_max_speed"].set_text("BRAKE")
        elif car.state["horizontal_velocity"] == car.properties["max_speed"]:
            self.annotations["brake_max_speed"].set_text("MAX SPEED")
        else:
            self.annotations["brake_max_speed"].set_text("")


    def animate(
        self, frame_generator, show_video=True,
        write_video=False, video_filepath=None, writer="ffmpeg",
        fps=50, writer_args=None, save_count=sys.maxsize
    ):
        """
        TODO

        Recommended VideoWriter settings:
            writer = "ffmpeg", fps = 1 / (time_step * generator_interval),
            writer_args = ["-vcodec", "h264"]
        where generator_interval refers to the interval (if any) chosen for
        the frame_generator function.
        """

        anim = matplotlib.animation.FuncAnimation(
            self.fig, self.update_animation, frames=frame_generator,
            interval=1, repeat=False, save_count=save_count
        )

        if write_video:
            if video_filepath is None:
                video_filepath = datetime.now().strftime("%Y%m%d%H%M%S_halfcar.mp4")
            writer_class = animation.writers[writer]
            writer_obj = writer_class(fps=fps, extra_args=writer_args)
            anim.save(video_filepath, writer=writer_obj)

        if show_video:
            plt.show()
