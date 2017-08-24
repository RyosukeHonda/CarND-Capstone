"""
Utilities used by dbw_py
"""

import numpy as np


def get_polynomial_fit(arguments, values, degree):
    """
    Given a list of arguments and values, fit a polynomial to them and return its coefficients
    :param arguments: list of styx_msgs.msg.Waypoint instances
    :param values: integer
    :param degree: polynomial degree
    :return: polynomial coefficients, list of floats starting from constant ending at last degree coefficient
    """

    coefficients = np.polyfit(arguments, values, degree)

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

    xs = [waypoint.pose.pose.position.x for waypoint in waypoints]
    ys = [waypoint.pose.pose.position.y for waypoint in waypoints]

    is_road_horizontal = is_road_more_horizontal_than_vertical(waypoints)

    arguments = xs if is_road_horizontal else ys
    values = ys if is_road_horizontal else xs

    coefficients = get_polynomial_fit(arguments, values, degree)

    pose_argument = current_pose.position.x if is_road_horizontal else current_pose.position.y
    expected_value = evaluate_polynomial(coefficients, pose_argument)

    actual_value = current_pose.position.y if is_road_horizontal else current_pose.position.x

    return actual_value - expected_value


def is_road_more_horizontal_than_vertical(waypoints):

    first_x = waypoints[0].pose.pose.position.x
    last_x = waypoints[-1].pose.pose.position.x

    first_y = waypoints[0].pose.pose.position.y
    last_y = waypoints[-1].pose.pose.position.y

    return abs(first_x - last_x) > abs(first_y - last_y)