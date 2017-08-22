"""
Utilities used by dbw_py
"""

import numpy as np


def get_polynomial_fit(waypoints, degree):
    """
    Given a list of waypoints, fit a polynomial to them and return its coefficients
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :param degree: integer
    :return: polynomial coefficients, list of floats starting from constant ending at last degree coefficient
    """

    xs = [waypoint.pose.pose.position.x for waypoint in waypoints]
    ys = [waypoint.pose.pose.position.y for waypoint in waypoints]

    coefficients = np.polyfit(xs, ys, degree)

    # Numpy returns coefficients with highest degree first, but we want them in opposite order
    return list(reversed(coefficients))