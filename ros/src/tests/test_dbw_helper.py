"""
Tests for dbw helper
"""

import styx_msgs.msg
import geometry_msgs.msg
import numpy as np

import src.twist_controller.dbw_helper as dbw_helper


def test_get_polynomial_fit_straight_line_with_constant_y():

    arguments = [0, 5, 10]
    values = [2, 2, 2]
    degree = 1

    expected = [2, 0]
    actual = dbw_helper.get_polynomial_fit(arguments, values, degree)

    assert np.allclose(expected, actual)


def test_get_polynomial_fit_straight_line_with_nonzero_slope():

    arguments = [0, 2, 4]
    values = [2, 6, 10]
    degree = 1

    expected = [2, 2]
    actual = dbw_helper.get_polynomial_fit(arguments, values, degree)

    assert np.allclose(expected, actual)


def test_evaluate_polynomial_first_order():

    coefficients = [2.5, 4.2]
    x = 1

    expected = 6.7
    actual = dbw_helper.evaluate_polynomial(coefficients, x)

    assert expected == actual


def test_evaluate_polynomial_second_order():

    coefficients = [2.5, 4.2, -1.2]
    x = 2

    expected = 6.1
    actual = dbw_helper.evaluate_polynomial(coefficients, x)

    assert np.isclose(expected, actual)


def test_get_waypoints_coordinates_matrix():

    first_waypoint = styx_msgs.msg.Waypoint()
    first_waypoint.pose.pose.position.x = 0
    first_waypoint.pose.pose.position.y = 2

    second_waypoint = styx_msgs.msg.Waypoint()
    second_waypoint.pose.pose.position.x = 8
    second_waypoint.pose.pose.position.y = 20

    third_waypoint = styx_msgs.msg.Waypoint()
    third_waypoint.pose.pose.position.x = 50
    third_waypoint.pose.pose.position.y = -4

    waypoints = [first_waypoint, second_waypoint, third_waypoint]

    expected = np.array([
        [0, 2],
        [8, 20],
        [50, -4]
    ])

    actual = dbw_helper.get_waypoints_coordinates_matrix(waypoints)

    assert np.all(expected == actual)