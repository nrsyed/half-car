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
        chassis = arc(x=front_well_center, y=well_center_height,
            r=well_radius, theta1=math.radians(-19.18),
            theta2=math.radians(200.96))

        rear_well_center = front_well_center - wheelbase
        chassis = np.concatenate((chassis,
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
                [chassis[0, -1] + delta_x],
                [chassis[1, -1] + delta_y]
            ])
            chassis = np.append(chassis, next_point, axis=1)

        # Add the first point to the end of the array to complete the loop.
        chassis = np.append(chassis, chassis[:, [0]], axis=1)

        # Add a third row of arbitrary values to the coordinates to make it
        # three-dimensional, allowing the use of 3D transformation matrices
        # and "3D-proofing" the implementation of the vehicle's appearance.
        chassis = np.vstack(
            (chassis, np.zeros((chassis.shape[1])))
        )

        # Set the vehicle COG and shift chassis such that the COG is at
        # (0, 0, 0). This simplifies things later.
        l_f = 0.4 * wheelbase
        l_r = 0.6 * wheelbase

        vehicle_COG = np.array([
            [rear_well_center + l_r],
            [0.4064],
            [0]
        ])

        chassis -= vehicle_COG

        # Similarly shift all wheel well coordinates and get wheel well position
        # vectors in the same coordinate reference frame as chassis.
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

        stiffness_matrix = (np.array([
            [-(k_fs + k_rs), l_r * k_rs - l_f * k_fs, k_fs, k_rs],
            [-(l_f * k_fs - l_r * k_rs), -(l_f**2 * k_fs + l_r**2 * k_rs),
                l_f * k_fs, -l_r * k_rs],
            [k_fs, l_f * k_fs, -(k_fs + k_ft), 0],
            [k_rs, -l_r * k_rs, 0, -(k_rs + k_rt)]])
            / mass_vector[:, None])

        damping_matrix = (np.array([
            [-(c_fs + c_rs), l_r * c_rs - l_f * c_fs, c_fs, c_rs],
            [-(l_f * c_fs - l_r * c_rs), -(l_f**2 * c_fs + l_r**2 * c_rs),
                l_f * c_fs, -l_r * c_rs],
            [c_fs, l_f * c_fs, -(c_fs + c_ft), 0],
            [c_rs, -l_r * c_rs, 0, -(c_rs + c_rt)]])
            / mass_vector[:, None])

        road_stiffness_matrix = (np.array([
            [0, 0],
            [0, 0],
            [k_ft, 0],
            [0, k_rt]])
            / mass_vector[:, None])

        road_damping_matrix = (np.array([
            [0, 0],
            [0, 0],
            [c_ft, 0],
            [0, c_rt]])
            / mass_vector[:, None])

        # Compute baseline height of COG above front wheel point of contact.
        lowest_point = np.amin(chassis[1,:])
        ground_clearance = 7 * 0.0254
        init_height = -lowest_point + ground_clearance

        # Set vehicle max speed in m/s, and max horizontal acceleration and
        # max horizontal deceleration (braking) in m/s^2. Max acceleration is
        # roughly based on a 0-60 mph time of 6.2 s.
        max_speed = 60
        max_accel = 4.4
        max_decel = -9.0

        # Store vehicle properties and appearance (coordinates) in dictionaries.
        self.appearance = {
            "chassis": chassis,
            "front_well_center": front_well_center,
            "rear_well_center": rear_well_center,
            "front_well_top": front_well_top,
            "rear_well_top": rear_well_top,
            "hub_radius": hub_radius,
            "wheel_radius": wheel_radius,
            "wheel": wheel,
            "hub": hub,
            "lowest_point": lowest_point,
            "ground_clearance": ground_clearance
        }

        self.properties = {
            "l_f": l_f,
            "l_r": l_r,
            "init_height": init_height,
            "wheelbase": wheelbase,
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
            "mass_vector": mass_vector,
            "stiffness_matrix": stiffness_matrix,
            "damping_matrix": damping_matrix,
            "road_stiffness_matrix": road_stiffness_matrix,
            "road_damping_matrix": road_damping_matrix,
            "max_speed": max_speed,
            "max_accel": max_accel,
            "max_decel": max_decel
        }

        # Initialize state vectors and other variables.
        self.state = {
            "position": np.zeros((4,1), dtype=np.float),
            "velocity": np.zeros((4,1), dtype=np.float),
            "accel": np.zeros((4,1), dtype=np.float),
            "road_position": np.zeros((2,1), dtype=np.float),
            "road_velocity": np.zeros((2,1), dtype=np.float),
            "horizontal_accel": 0,
            "horizontal_velocity": 0,
            "distance_traveled": 0
        }


    # TODO: allow independent gas and brake, fix gas and brake.

    def set_accel(self, accel):
        """
        Manually set car's horizontal acceleration in m/s^2.
        """

        max_accel = self.properties["max_accel"]
        max_decel = self.properties["max_decel"]
        if max_decel <= accel <= max_accel:
            self.state["horizontal_accel"] = accel


    def set_velocity(self, velocity):
        """
        Manually set car's horizontal velocity in m/s.
        """

        if 0 <= velocity <= self.properties["max_speed"]:
            self.state["horizontal_velocity"] = velocity


    def gas(self, accel, units="mps"):
        """
        Set the car's horizontal acceleration in m/s^2 (i.e.,
        press the gas pedal) or as a fraction (0 <= accel <= 1) of
        max acceleration.
        """

        max_accel = self.properties["max_accel"]
        if accel >= 0:
            if units == "fraction":
                new_accel = accel * max_accel
            else:
                new_accel = accel
            self.state["horizontal_accel"] = min(new_accel, max_accel)


    def brake(self, decel, units="mps"):
        """
        Set the car's horizontal deceleration in m/s^2 (i.e.,
        press the brake pedal) or as a fraction (0 <= decel <= 1) of
        max deceleration. Note that deceleration is negative.
        """

        max_decel = self.properties["max_decel"]
        if units == fraction and decel >= 0:
            new_decel = decel * max_decel
        #elif decel 

    def update_state(self, time_step): 
        position = self.state["position"]
        velocity = self.state["velocity"]
        accel = self.state["accel"]
        road_position = self.state["road_position"]
        road_velocity = self.state["road_velocity"]

        # TODO


        """
        car.accelerationVector = (car.stiffnessMatrix.dot(car.positionVector)
            + car.dampingMatrix.dot(car.velocityVector)
            + car.roadStiffnessMatrix.dot(car.roadPositionVector)
            + car.roadDampingMatrix.dot(car.roadVelocityVector)
            + car.normal_force_vector())

        # Clamp pitch angle to +/- 5 deg.
        if car.positionVector[1] > math.radians(5):
            car.positionVector[1] = math.radians(5)
            car.velocityVector[1] = 0
        elif car.positionVector[1] < math.radians(-5):
            car.positionVector[1] = math.radians(-5)
            car.velocityVector[1] = 0
        """

    @property
    def normal_force_vector(self):
        init_height = self.properties["init_height"]
        wheelbase = self.properties["wheelbase"]
        position = self.state["position"]
        horizontal_accel = self.state["horizontal_accel"]
        m = self.properties["m"]
        mass_vector = self.properties["mass_vector"]

        # Obtain the height of the COG above the front wheel point of contact,
        # i.e., the height of the COG about the driving force vector (assuming
        # a front-wheel drive vehicle).
        height = init_height + position[0] + position[2]

        normal_force_vector = (np.array([
            [0],
            [0],
            [height * m * horizontal_accel / wheelbase],
            [-height * m * horizontal_accel / wheelbase]
            ])
            / mass_vector[:, None]
        )

        return normal_force_vector
