import math
import numpy as np


PI = math.pi


def arc(center=(0, 0), radius=1, theta1=0, theta2=2*PI, resolution=180):
    """
    Return x and y coordinates of a circular arc.

    :param center: x and y coordinates of the center of the arc.
    :type center: tuple of (float, float)

    :param radius: Radius of the arc.
    :type radius: float

    :param theta1: Angle of beginning of arc in radians.
    :type theta1: float

    :param theta2: Angle of end of arc in radians.
    :type theta2: float

    :param resolution: number of points per 2*PI radians.
    :type resolution: int

    :return: x, y coordinates of arc.
    :rtype: np.ndarray
    """

    x, y = center
    thetas = np.linspace(theta1, theta2,
        int(abs(theta2 - theta1) * (resolution / (2 * PI)))
    )
    return np.vstack((x + radius * np.cos(thetas), y + radius * np.sin(thetas)))


def zigzag(start, end, nodes, width):
    """
    Return a list of points corresponding to a zigzag.

    :param start: (x, y) coordinates of the first endpoint.
    :type start: tuple of (array-like, array-like)

    :param end: (x, y) coordinates of the second endpoint.
    :type start: tuple of (array-like, array-like)

    :param nodes: Number of zigzag "nodes".
    :type nodes: int

    :param width: Width of the zigzag.
    :type width: float

    :return: An array of x coordinates and an array of y coordinates.
    :rtype: np.ndarray, np.ndarray
    """

    # Check that nodes is at least 1.
    nodes = max(int(nodes), 1)

    # Convert to numpy array to account for inputs of different types/shapes.
    start, end = np.array(start).reshape((2,)), np.array(end).reshape((2,))

    # If both points are coincident, return the x and y coords of one of them.
    if (start == end).all():
        return start[0], start[1]

    # Calculate length of zigzag (distance between endpoints).
    length = np.linalg.norm(np.subtract(end, start))

    # Calculate unit vectors tangent (u_t) and normal (u_t) to zigzag.
    u_t = np.subtract(end, start) / length
    u_n = np.array([[0, -1], [1, 0]]).dot(u_t)

    # Initialize array of x (row 0) and y (row 1) coords of the nodes+2 points.
    zigzag_coords = np.zeros((2, nodes + 2))
    zigzag_coords[:,0], zigzag_coords[:,-1] = start, end

    # Check that length is not greater than the total length the zigzag
    # can extend (otherwise, math domain error will result), and compute the
    # normal distance from the centerline of the zigzag.
    normal_dist = math.sqrt(max(0, width**2 - (length**2 / nodes**2))) / 2

    # Compute the coordinates of each point (each node).
    for i in range(1, nodes + 1):
        zigzag_coords[:,i] = (
            start
            + ((length * (2 * i - 1) * u_t) / (2 * nodes))
            + (normal_dist * (-1)**i * u_n))

    return zigzag_coords[0,:], zigzag_coords[1,:]
