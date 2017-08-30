"""
Utilities used by waypoints_updater
"""

import numpy as np


def get_waypoints_matrix(waypoints):
    """
    Converts waypoints listt to numpy matrix
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: 2D numpy array
    """

    waypoints_matrix = np.zeros(shape=(len(waypoints), 2), dtype=np.float32)

    for index, waypoint in enumerate(waypoints):
        waypoints_matrix[index, 0] = waypoint.pose.pose.position.x
        waypoints_matrix[index, 1] = waypoint.pose.pose.position.y

    return waypoints_matrix


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


def get_closest_waypoint_index(position, waypoints_matrix):
    """
    Given a pose and waypoints list, return index of waypoint closest to pose
    :param position: geometry_msgs.msgs.Position instance
    :param waypoints_matrix: numpy matrix with waypoints coordinates
    :return: integer index
    """

    x_distances = waypoints_matrix[:, 0] - position.x
    y_distances = waypoints_matrix[:, 1] - position.y

    squared_distances = x_distances**2 + y_distances**2
    return np.argmin(squared_distances)


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


def get_smoothed_out_waypoints(waypoints):
    """
    Return smoothed out waypoints. Waypoints are smoothed out and evenly spaced out.
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: list of styx_msgs.msg.Waypoint instances
    """

    xs = [waypoint.pose.pose.position.x for waypoint in waypoints]
    ys = [waypoint.pose.pose.position.y for waypoint in waypoints]

    # We will use indices as arguments when computing smoothing out function.
    # This way we will avoid problems of steep functions if xs changed very little for large changes in ys
    indices = list(range(len(waypoints)))

    degree = 3
    x_poly = np.polyfit(indices, xs, degree)
    y_poly = np.polyfit(indices, ys, degree)

    x_values = np.polyval(x_poly, indices)
    y_values = np.polyval(y_poly, indices)

    smooth_waypoints = []

    for index, waypoint in enumerate(waypoints):

        waypoint.pose.pose.position.x = x_values[index]
        waypoint.pose.pose.position.y = y_values[index]

        smooth_waypoints.append(waypoint)

    return smooth_waypoints


def save_waypoints(waypoints, path):

    waypoints_matrix = get_waypoints_matrix(waypoints)
    np.savetxt(path, waypoints_matrix)

