"""
Utilities used by waypoints_updater
"""

import numpy as np


def get_distance_between_points(first, second):
    """
    Return distance between two points
    :param first: geometry_msgs.msgs.Point instance
    :param second: geometry_msgs.msgs.Point instance
    :return: float
    """

    x_difference = first.x - second.x
    y_difference = first.y - second.y

    return np.sqrt(x_difference**2 + y_difference**2)


def get_closest_waypoint_index(pose, waypoints):
    """
    Given a pose and waypoints list, return index of waypoint closest to pose
    :param pose: geometry_msgs.msgs.Pose instance
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: integer index
    """

    best_index = 0
    best_distance = get_distance_between_points(pose.position, waypoints[0].pose.pose.position)

    for index, waypoint in enumerate(waypoints):

        distance = get_distance_between_points(pose.position, waypoint.pose.pose.position)

        if distance < best_distance:

            best_index = index
            best_distance = distance

    return best_index


def get_sublist(elements, start_index, size):
    """
    Given a list of elements, start index and size of sublist, returns
    sublist starting from start_index that has size elements. Takes care of wrapping around should
    start_index + size > len(elements)
    :param elements: list
    :param start_index: start index
    :param size: size of sublist
    :return: sublist, wrapped around beginning of elements list if necessary
    """

    # A very simple, not necessarily efficient solution
    doubled_elements = elements + elements[:size]
    return doubled_elements[start_index: start_index + size]
