"""
Tests for waypoints helper
"""

import styx_msgs.msg
import geometry_msgs.msg
import numpy as np

import src.waypoint_updater.waypoints_helper as waypoints_helper


def test_get_distance_between_points():

    first = geometry_msgs.msg.Point()
    first.x = 0
    first.y = 0

    second = geometry_msgs.msg.Point()
    second.x = 10
    second.y = 20

    expected = np.sqrt(500)
    actual = waypoints_helper.get_distance_between_points(first, second)

    assert np.isclose(expected, actual)


def test_get_closest_waypoint_index():

    pose = geometry_msgs.msg.Pose()
    pose.position.x = 10
    pose.position.y = 10

    first_waypoint = styx_msgs.msg.Waypoint()
    first_waypoint.pose.pose.position.x = 0
    first_waypoint.pose.pose.position.y = 0

    second_waypoint = styx_msgs.msg.Waypoint()
    second_waypoint.pose.pose.position.x = 8
    second_waypoint.pose.pose.position.y = 8

    third_waypoint = styx_msgs.msg.Waypoint()
    third_waypoint.pose.pose.position.x = 20
    third_waypoint.pose.pose.position.y = 20

    waypoints = [first_waypoint, second_waypoint, third_waypoint]

    assert 1 == waypoints_helper.get_closest_waypoint_index(pose, waypoints)