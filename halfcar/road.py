from collections import deque
import math
import numpy as np

class Road:
    def __init__(
        self, length, resolution=300, mode="flat", amplitude=0.3,
        frequency=0.04, x_min=None
    ):

        """
        length: amount of road (meters)
        resolution: points per meter
        """

        num_points = int(length * resolution)

        if x_min == None:
            x_min = 0
        x_max = x_min + length

        self.x_coords = np.linspace(x_min, x_max, num_points)
        self.y_coords = deque(np.zeros((num_points,)), maxlen=num_points)
        self.resolution = resolution
        self.mode = mode
        self.num_points = num_points
        self.distance = 0

        self.amplitude = amplitude
        self.frequency = frequency

    def generate(self, length_to_generate):
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

            if self.mode in ("sine", "square"):
                sine_value = math.sin(frequency * (distance + (i / resolution)))
                if self.mode == "sine":
                    next_point = amplitude * sine_value
                elif self.mode == "square":
                    if sine_value >= 0:
                        next_point = 0
                    else:
                        next_point = amplitude
            elif self.mode == "flat":
                next_point = 0
            #TODO: other modes

            self.y_coords.append(next_point)
        self.distance += length_to_generate
        return (self.x_coords, self.y_coords)

    def __call__(self, length_to_generate=None):
        """
        TODO
        If the class instance is called without an argument, the current road
        profile (x_coords, y_coords) is returned. Otherwise, this method acts
        as a wrapper for `Road.generate()`.
        """
        if length_to_generate:
            return self.generate(length_to_generate)
        else:
            return (self.x_coords, self.y_coords)
