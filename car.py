import math
import numpy as np

PI = math.pi
def arc(x=0, z=0, r=1, theta1=0, theta2=PI, resolution=180):
    '''Returns x and z coords (row 0 and row 1, respectively)
        of arc. "resolution" = number of points per 2*PI rads.
        Input arguments x and z refer to centerpoint of arc.
    '''
    thetas = np.linspace(theta1, theta2,
        int(abs(theta2 - theta1) * (resolution / (2*PI))))
    return np.vstack((x + r*np.cos(thetas), z + r*np.sin(thetas)))

class Car:
    def __init__(self):
        
        # Wheel well dimensions.
        wheelbase = 2.74
        wellCenterHeight = 0.124847
        wellRadius = 0.38

        # Front wheel well.
        xFrontWellCenter = -0.358914
        chassisCoords = arc(x=xFrontWellCenter, z=wellCenterHeight,
            r=wellRadius, theta1=math.radians(-19.18),
            theta2=math.radians(200.96))

        # Rear wheel well.
        xRearWellCenter = xFrontWellCenter - wheelbase
        chassisCoords = np.concatenate((chassisCoords,
            arc(x=xRearWellCenter, z=wellCenterHeight, r=wellRadius,
            theta1=math.radians(-19.18), theta2=math.radians(194.3))), axis=1)

        # Car profile excluding wheel wells.
        lines = np.array([
            [-.514, -.069, .269, .392, 1.03, .891, .583, 1.32, .138, -.092],
            [0, .392, .415, .046, .292, 0, -.33, -.253, -.238, -.353]])

        for i in range(len(lines[0,:])):
            chassisCoords = np.append(
                chassisCoords, chassisCoords[:,[-1]] + lines[:,[i]], axis=1)
        chassisCoords = np.append(chassisCoords, chassisCoords[:,[0]], axis=1)
        chassisCoords = np.vstack(
            (chassisCoords, np.ones((len(chassisCoords[0,:]),))))

        # Chassis COG location.
        chassisCOG = np.array([
            [xRearWellCenter + (0.6 * wheelbase)], [0.4064], [1]])
        
        # Shift chassis coords so COG is (0, 0).
        chassisCoords[:2,:] -= chassisCOG[:2,:]
        self.chassisCoords = chassisCoords
        l_f = 0.4 * wheelbase
        l_r = 0.6 * wheelbase

        # Get shifted wheel well coordinates.
        self.frontWellCenter = (np.array([[xFrontWellCenter], [wellCenterHeight], [1]])
            - chassisCOG)
        self.rearWellCenter = (np.array([[xRearWellCenter], [wellCenterHeight], [1]])
            - chassisCOG)
        self.frontWellTop = self.frontWellCenter + np.array([[0], [wellRadius], [1]])
        self.rearWellTop = self.rearWellCenter + np.array([[0], [wellRadius], [1]])

        # Wheel.
        hubDiameter = 17 * .0254    # in meters
        tireWidth = 0.225
        tireAspect = 0.5
        tireHeight = tireAspect * tireWidth
        self.hubRadius = 0.5 * hubDiameter
        self.wheelRadius = self.hubRadius + tireHeight
        self.wheel = arc(r=self.wheelRadius, theta2=2*PI)
        self.hub = arc(r=self.hubRadius, theta2=2*PI)

        self.wheelbase = wheelbase

        # Mass, inertia, stiffness, and damping properties.
        #m_c = 1350
        m_c = 1600
        m_f = 2 * 23
        m_r = m_f
        I_yy = 2500
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

        # NOTE: matrices and vectors below are based on the following
        # solution vector: {z_c, phi, z_f, z_r}

        massVector = np.array([m_c, I_yy, m_f, m_r])

        self.stiffnessMatrix = (np.array([
            [-(k_fs + k_rs), l_r * k_rs - l_f * k_fs, k_fs, k_rs],
            [-(l_f * k_fs - l_r * k_rs), -(l_f**2 * k_fs + l_r**2 * k_rs),
                l_f * k_fs, -l_r * k_rs],
            [k_fs, l_f * k_fs, -(k_fs + k_ft), 0],
            [k_rs, -l_r * k_rs, 0, -(k_rs + k_rt)]])
            / massVector[:, None])

        self.dampingMatrix = (np.array([
            [-(c_fs + c_rs), l_r * c_rs - l_f * c_fs, c_fs, c_rs],
            [-(l_f * c_fs - l_r * c_rs), -(l_f**2 * c_fs + l_r**2 * c_rs),
                l_f * c_fs, -l_r * c_rs],
            [c_fs, l_f * c_fs, -(c_fs + c_ft), 0],
            [c_rs, -l_r * c_rs, 0, -(c_rs + c_rt)]])
            / massVector[:, None])

        self.roadStiffnessMatrix = (np.array([
            [0, 0],
            [0, 0],
            [k_ft, 0],
            [0, k_rt]])
            / massVector[:, None])

        self.roadDampingMatrix = (np.array([
            [0, 0],
            [0, 0],
            [c_ft, 0],
            [0, c_rt]])
            / massVector[:, None])

        self.m_c = m_c
        self.m_f = m_f
        self.m_r = m_r
        self.I_yy = I_yy
        self.m = m

        self.k_fs = k_fs
        self.k_rs = k_rs
        self.k_ft = k_ft
        self.k_rt = k_rt

        self.c_fs = c_fs
        self.c_rs = c_rs
        self.c_ft = c_ft
        self.c_rt = c_rt

        self.massVector = massVector

        # Initialize vector arrays and other variables.
        self.positionVector = np.zeros((4,1), dtype=np.float)
        self.velocityVector = np.zeros((4,1), dtype=np.float)
        self.accelerationVector = np.zeros((4,1), dtype=np.float)
        self.roadPositionVector = np.zeros((2,1), dtype=np.float)
        self.roadVelocityVector = np.zeros((2,1), dtype=np.float)

        # Initialize variables for motion in x direction.
        self.horizontalAccel = 0
        self.horizontalVelocity = 0
        self.distTraveled = 0

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
