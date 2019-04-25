import math
import numpy as np


PI = math.pi


def arc(x=0, y=0, r=1, theta1=0, theta2=PI, resolution=180):
    #TODO: revisit docstrings
    """
    Returns x and y coords (row 0 and row 1, respectively)
    of arc. "resolution" = number of points per 2*PI rads.
    Input arguments x and y refer to centerpoint of arc.
    """

    thetas = np.linspace(theta1, theta2,
        int(abs(theta2 - theta1) * (resolution / (2*PI))))
    return np.vstack((x + r*np.cos(thetas), y + r*np.sin(thetas)))


class Car:
    def __init__(self):

        # Develop the profile of the vehicle chassis. All dimensions are in
        # meters. A number of dimensions were either obtained empirically
        # or estimated from photographs. The "initial" coordinate system
        # is based on the vehicle facing right, with the positive x axis
        # extending to the right and the positive y axis extending up.
        # x=0 is at the right corner of the front wheel well, and z=0 is at
        # ground level.
        # TODO: pictures
        
        # Wheel well and related dimensions.
        wheelbase = 2.74
        well_center_height = 0.124847
        well_radius = 0.38

        front_well_center = -0.358914
        chassis_coords = arc(x=front_well_center, y=well_center_height,
            r=well_radius, theta1=math.radians(-19.18),
            theta2=math.radians(200.96))

        rear_well_center = front_well_center - wheelbase
        chassis_coords = np.concatenate((chassis_coords,
            arc(x=rear_well_center, y=well_center_height, r=well_radius,
            theta1=math.radians(-19.18), theta2=math.radians(194.3))), axis=1)

        # The remaining chassis profile is developed by traveling from the left
        # corner of the rear wheel well clockwise until arriving at the right
        # corner of the front wheel well. The array chassis_point_deltas
        # contains the change in x (first row) and change in y (second row)
        # from one point to the next, i.e., the first ordered pair
        # (-.514, 0) indicates that the first chassis point is located .514 m
        # left of the corner of the rear wheel well and at the same height.
        # The second pair of coordinates (-.069, .392) indicates that the
        # next point, which corresponds to the rear bumper, is located .069 m
        # left of the previous point and .392 m above it.
        # TODO: picture worth a thousand words.
        chassis_deltas = [
            [-.514, -.069, .269, .392, 1.03, .891, .583, 1.32, .138, -.092],
            [0, .392, .415, .046, .292, 0, -.33, -.253, -.238, -.353]
        ]

        for delta_x, delta_y in zip(*chassis_deltas):
            next_point = np.array([
                [chassis_coords[0, -1] + delta_x],
                [chassis_coords[1, -1] + delta_y]
            ])
            chassis_coords = np.append(chassis_coords, next_point, axis=1)

        # Add the first point to the end of the array to complete the loop.
        chassis_coords = np.append(chassis_coords, chassis_coords[:, [0]], axis=1)

        # Add a third row of arbitrary values to the coordinates to make it
        # three-dimensional, allowing the use of 3D transformation matrices
        # and "3D-proofing" the implementation of the vehicle's appearance.
        chassis_coords = np.vstack(
            (chassis_coords, np.zeros((chassis_coords.shape[1])))
        )

        # Set the vehicle COG and shift chassis_coords such that the COG is at
        # (0, 0, 0). This simplifies things later.
        l_f = 0.4 * wheelbase
        l_r = 0.6 * wheelbase

        vehicle_COG = np.array([
            [rear_well_center + l_r],
            [0.4064],
            [0]
        ])

        chassis_coords -= vehicle_COG
        self.chassis_coords = chassis_coords
        self.wheelbase = wheelbase

        # Similarly shift all wheel well coordinates and get wheel well position
        # vectors in the same coordinate reference frame as chassis_coords.
        front_well_center = np.array([
            [front_well_center],
            [well_center_height],
            [0]
        ])

        rear_well_center = np.array([
            [rear_well_center],
            [well_center_height],
            [0]
        ])

        front_well_center -= vehicle_COG
        rear_well_center -= vehicle_COG

        front_well_top = front_well_center + np.array([[0], [well_radius], [0]])
        rear_well_top = rear_well_center + np.array([[0], [well_radius], [0]])

        self.front_well_center = front_well_center
        self.rear_well_center = rear_well_center
        self.front_well_top = front_well_top
        self.rear_well_top = rear_well_top

        # Wheel-related dimensions based on 2010 Accord EX-L stock tire type
        # P225/50R17 (225 mm width, 50% aspect ratio, 17 inch hub diameter).
        tire_width = 0.225
        tire_aspect = 0.50
        hub_diameter = 17 * 0.0254      # convert in to m

        tire_height = tire_aspect * tire_width
        hub_radius = 0.5 * hub_diameter
        wheel_radius = hub_radius + tire_height
        wheel = arc(r=wheel_radius, theta2=2*PI)
        hub = arc(r=hub_radius, theta2=2*PI)

        self.hub_radius = hub_radius
        self.wheel_radius = wheel_radius
        self.wheel = wheel
        self.hub = hub

        # Mass, inertia, stiffness, and damping properties.
        #m_c = 1350
        m_c = 1600
        m_f = 2 * 23
        m_r = m_f
        I_zz = 2500
        m = m_c + m_f + m_r

        k_fs = 80000
        #k_rs = (l_r / l_f) * k_fs
        k_rs = 1.1 * k_fs
        k_ft = 150000
        k_rt = 150000

        c_fs = 1000
        c_rs = 1000
        c_ft = 20
        c_rt = 20

         """
         The matrices and vectors below are based on the solution
         vector {y_c, phi, y_f, y_r}. As a numpy array:
         np.array([
            [y_c],
            [phi],
            [y_f],
            [y_r]
        ])
        """
          

        mass_vector = np.array([m_c, I_zz, m_f, m_r])

        self.stiffness_matrix = (np.array([
            [-(k_fs + k_rs), l_r * k_rs - l_f * k_fs, k_fs, k_rs],
            [-(l_f * k_fs - l_r * k_rs), -(l_f**2 * k_fs + l_r**2 * k_rs),
                l_f * k_fs, -l_r * k_rs],
            [k_fs, l_f * k_fs, -(k_fs + k_ft), 0],
            [k_rs, -l_r * k_rs, 0, -(k_rs + k_rt)]])
            / mass_vector[:, None])

        self.damping_matrix = (np.array([
            [-(c_fs + c_rs), l_r * c_rs - l_f * c_fs, c_fs, c_rs],
            [-(l_f * c_fs - l_r * c_rs), -(l_f**2 * c_fs + l_r**2 * c_rs),
                l_f * c_fs, -l_r * c_rs],
            [c_fs, l_f * c_fs, -(c_fs + c_ft), 0],
            [c_rs, -l_r * c_rs, 0, -(c_rs + c_rt)]])
            / mass_vector[:, None])

        self.road_stiffness_matrix = (np.array([
            [0, 0],
            [0, 0],
            [k_ft, 0],
            [0, k_rt]])
            / mass_vector[:, None])

        self.road_damping_matrix = (np.array([
            [0, 0],
            [0, 0],
            [c_ft, 0],
            [0, c_rt]])
            / mass_vector[:, None])

        self.properties = {
            "m_c": m_c,
            "m_f": m_f,
            "m_r": m_r,
            "I_zz": I_zz,
            "m": m,
            "k_fs": k_fs,
            "k_rs": k_rs,
            "k_ft": k_ft,
            "k_rt": k_rt,
            "c_fs": c_fs,
            "c_rs": c_rs,
            "c_ft": c_ft,
            "c_rt": c_rt,
            "mass_vector": mass_vector
        }

        # Initialize state vectors and other variables.
        self.state = {
            "position": np.zeros((4,1), dtype=np.float),
            "velocity": np.zeros((4,1), dtype=np.float),
            "acceleration": np.zeros((4,1), dtype=np.float),
            "road_position": np.zeros((2,1), dtype=np.float),
            "road_velocity": np.zeros((2,1), dtype=np.float),
            "horizontal_acceleration": 0,
            "horizontal_velocity": 0,
            "distance_traveled": 0
        }

        """
        # Initialize height of COG above front wheel point of contact.
        self.lowestPoint = np.amin(self.chassisCoords[1,:])
        self.groundClearance = 7 * 0.0254
        self.h_0 = -self.lowestPoint + self.groundClearance

        self.l_f = l_f
        self.l_r = l_r

    def normal_force_vector(self):
        h = self.h_0 + self.positionVector[0] - self.positionVector[2]
        normalForceVector = (np.array([
            [0],
            [0],
            [h * self.m * self.horizontalAccel / self.wheelbase],
            [-h * self.m * self.horizontalAccel / self.wheelbase]])
            / self.massVector[:, None])
        return normalForceVector
        """
