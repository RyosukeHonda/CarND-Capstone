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


def test_get_normalized_angle_normal_range():

    angle = 0.1

    expected = 0.1
    actual = dbw_helper.get_normalized_angle(angle)

    assert np.isclose(expected, actual)


def test_get_normalized_angle_above_normal_rangle():

    angle = 3

    expected = 3 - np.pi
    actual = dbw_helper.get_normalized_angle(angle)

    assert np.isclose(expected, actual)


def test_get_normalized_angle_below_normal_rangle():

    angle = -2

    expected = -2 + np.pi
    actual = dbw_helper.get_normalized_angle(angle)

    assert np.isclose(expected, actual)


def test_get_arc_angle_positive_90_deg_angle():

    a = geometry_msgs.msg.Pose()
    a.position.x = 0
    a.position.y = 10

    b = geometry_msgs.msg.Pose()
    b.position.x = 0
    b.position.y = 0

    c = geometry_msgs.msg.Pose()
    c.position.x = 10
    c.position.y = 0

    expected = np.pi / 2.0
    actual = dbw_helper.get_arc_angle(a, b, c)

    assert np.isclose(actual, expected)


def test_get_arc_angle_negative_90_deg_angle():

    a = geometry_msgs.msg.Pose()
    a.position.x = 0
    a.position.y = -10

    b = geometry_msgs.msg.Pose()
    b.position.x = 0
    b.position.y = 0

    c = geometry_msgs.msg.Pose()
    c.position.x = 10
    c.position.y = 0

    expected = -np.pi / 2.0
    actual = dbw_helper.get_arc_angle(a, b, c)

    assert np.isclose(actual, expected)


def test_get_arc_angle_positive_30_deg_angle_b_at_origin():

    a = geometry_msgs.msg.Pose()
    a.position.x = 10
    a.position.y = 17.32

    b = geometry_msgs.msg.Pose()
    b.position.x = 0
    b.position.y = 0

    c = geometry_msgs.msg.Pose()
    c.position.x = 10
    c.position.y = 5.77

    expected = np.pi / 6.0
    actual = dbw_helper.get_arc_angle(a, b, c)

    assert np.isclose(actual, expected, atol=0.01)


def test_get_arc_angle_positive_30_deg_angle_b_shifted_from_origin():

    a = geometry_msgs.msg.Pose()
    a.position.x = 15
    a.position.y = 27.32

    b = geometry_msgs.msg.Pose()
    b.position.x = 5
    b.position.y = 10

    c = geometry_msgs.msg.Pose()
    c.position.x = 15
    c.position.y = 15.77

    expected = np.pi / 6.0
    actual = dbw_helper.get_arc_angle(a, b, c)

    assert np.isclose(actual, expected, atol=0.01)


def test_get_arc_angle_negative_30_deg_angle_b_at_origin():

    a = geometry_msgs.msg.Pose()
    a.position.x = 10
    a.position.y = -17.32

    b = geometry_msgs.msg.Pose()
    b.position.x = 0
    b.position.y = 0

    c = geometry_msgs.msg.Pose()
    c.position.x = 10
    c.position.y = -5.77

    expected = -np.pi / 6.0
    actual = dbw_helper.get_arc_angle(a, b, c)

    assert np.isclose(actual, expected, atol=0.01)


def test_get_arc_angle_negative_30_deg_angle_b_shifted_from_origin():

    a = geometry_msgs.msg.Pose()
    a.position.x = -10
    a.position.y = -27.32

    b = geometry_msgs.msg.Pose()
    b.position.x = -20
    b.position.y = -10

    c = geometry_msgs.msg.Pose()
    c.position.x = -10
    c.position.y = -15.77

    expected = -np.pi / 6.0
    actual = dbw_helper.get_arc_angle(a, b, c)

    assert np.isclose(actual, expected, atol=0.01)