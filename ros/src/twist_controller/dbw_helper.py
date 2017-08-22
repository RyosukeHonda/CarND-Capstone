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


def evaluate_polynomial(coefficients, x):
    """
    Given polynomial coefficients, evaluate polynomial at point x
    :param coefficients:
    :param x:
    :return:
    """

    y = 0

    for power, coefficient in enumerate(coefficients):

        y += coefficient * (x ** power)

    return y


def get_cross_track_error(waypoints, current_pose):
    """
    Given waypoints ahead of the car, fist polynomial to them, estimates expected y at current x pose and compares
    that to actual y to compute cross track error - a deviation from expected trajectory
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :param current_pose: geometry_msgs.msgs.Pose instance
    :return: float
    """

    degree = 2
    coefficients = get_polynomial_fit(waypoints, degree)

    expected_y = evaluate_polynomial(coefficients, current_pose.position.x)
    actual_y = current_pose.position.y

    return actual_y - expected_y
