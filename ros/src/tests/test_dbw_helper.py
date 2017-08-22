"""
Tests for dbw helper
"""

import styx_msgs.msg
import numpy as np

import src.twist_controller.dbw_helper as dbw_helper


def test_get_polynomial_fit_straight_line_with_constant_y():

    first_waypoint = styx_msgs.msg.Waypoint()
    first_waypoint.pose.pose.position.x = 0
    first_waypoint.pose.pose.position.y = 2

    second_waypoint = styx_msgs.msg.Waypoint()
    second_waypoint.pose.pose.position.x = 5
    second_waypoint.pose.pose.position.y = 2

    third_waypoint = styx_msgs.msg.Waypoint()
    third_waypoint.pose.pose.position.x = 10
    third_waypoint.pose.pose.position.y = 2

    waypoints = [first_waypoint, second_waypoint, third_waypoint]
    degree = 1

    expected = [2, 0]
    actual = dbw_helper.get_polynomial_fit(waypoints, degree)

    assert np.allclose(expected, actual)


def test_get_polynomial_fit_straight_line_with_nonzero_slope():

    first_waypoint = styx_msgs.msg.Waypoint()
    first_waypoint.pose.pose.position.x = 0
    first_waypoint.pose.pose.position.y = 2

    second_waypoint = styx_msgs.msg.Waypoint()
    second_waypoint.pose.pose.position.x = 2
    second_waypoint.pose.pose.position.y = 6

    third_waypoint = styx_msgs.msg.Waypoint()
    third_waypoint.pose.pose.position.x = 4
    third_waypoint.pose.pose.position.y = 10

    waypoints = [first_waypoint, second_waypoint, third_waypoint]
    degree = 1

    expected = [2, 2]
    actual = dbw_helper.get_polynomial_fit(waypoints, degree)

    assert np.allclose(expected, actual)