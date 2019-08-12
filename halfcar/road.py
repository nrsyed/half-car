from collections import deque
import math
import numpy as np

class Road:
    def __init__(
        self, length, resolution=300, mode="flat", amplitude=0.3,
        frequency=0.04, x_min=None
    ):

        """
        Callable class that generates road x and y coordinates for a specified.
        road profile.

        Essentially, the road profile is a function defined for all x. This
        class effectively slides a window of length `length` along that
        profile, computing the y coordinates for a given window and storing
        them in the deque `Road.y_coords`.

        :param length: Length of road segment in meters.
        :type length: float

        :param resolution: Number of points per meter.
        :type resolution: int

        :param mode: Road profile mode: "flat", "sine", "square", "triangle", "bump".
        :type mode: str

        :param amplitude: Amplitude of sine or square wave.
        :type amplitude: float

        :param frequency: Frequency of sine or square wave.
        :type frequency: float

        :param x_min: Minimum x coordinate.
        :type x_min: float
        """

        # Number of points in the road profile.
        num_points = int(length * resolution)

        if x_min == None:
            x_min = -length / 2
        x_max = x_min + length

        # x coordinates will not change. Store y coordinates in deque.
        self.x_coords = np.linspace(x_min, x_max, num_points)
        self.y_coords = deque(np.zeros((num_points,)), maxlen=num_points)
        self.resolution = resolution
        self.mode = mode
        self.num_points = num_points

        # Total length of road generated, i.e., distance traveled.
        self.distance = 0

        self.amplitude = amplitude
        self.frequency = frequency

    def generate(self, length_to_generate):
        """
        Generate a new road profile based on the distance, i.e. length,
        traveled since the previous road profile.

        :param length_to_generate: Distance traveled since last update.
        :type length_to_generate: float

        :return: An array of x coordinates and an array of y coordinates.
        :rtype: (array_like, array_like)
        """

        # Number of new points to generate.
        num_new_points = int(length_to_generate * self.resolution)

        # If a nonzero length_to_generate is requested but num_new_points
        # rounds to zero, generate at least one point.
        if length_to_generate > 0 and num_new_points == 0:
            num_new_points = 1

        amplitude = self.amplitude
        frequency = self.frequency
        distance = self.distance
        resolution = self.resolution

        for i in range(num_new_points):
            self.y_coords.popleft()

            if self.mode in ("sine", "square", "triangle", "bump"):
                sin_arg = frequency * (distance + (i / resolution))
                sine_value = math.sin(sin_arg)
                if self.mode == "sine":
                    next_point = amplitude * sine_value
                elif self.mode == "square":
                    if sine_value >= 0:
                        next_point = 0
                    else:
                        next_point = amplitude
                elif self.mode in ("triangle", "bump"):
                    wave_arg = sin_arg % (2 * math.pi)
                    if self.mode == "bump":
                        wave_arg *= 2
                    if wave_arg <= math.pi:
                        # Rising line
                        next_point = amplitude * wave_arg / math.pi
                    elif wave_arg <= 2 * math.pi:
                        # Rising line
                        next_point = amplitude * (2 * math.pi - wave_arg) / math.pi
                    else:
                        next_point = 0
            elif self.mode == "flat":
                next_point = 0
            else:
                raise ValueError(f"Invalid {self.mode}")

            self.y_coords.append(next_point)

        self.distance += length_to_generate
        return (self.x_coords, self.y_coords)

    def __call__(self, length_to_generate=None):
        """
        If the class instance is called without an argument, the current road
        profile (x_coords, y_coords) is returned. Otherwise, this method acts
        as a wrapper for `Road.generate()`.
        """

        if length_to_generate:
            return self.generate(length_to_generate)
        else:
            return (self.x_coords, self.y_coords)
