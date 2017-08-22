"""
Tests for waypoints helper
"""

import src.waypoint_updater.waypoints_helper as waypoints_helper


def test_simple():

    assert 0 == waypoints_helper.get_closest_waypoint_index(1, 2)