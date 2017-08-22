"""
Utilities used by waypoints_updater
"""

import numpy as np


def get_distance_between_points(first, second):

    x_difference = first.x - second.x
    y_difference = first.y - second.y

    return np.sqrt(x_difference**2 + y_difference**2)


def get_closest_waypoint_index(pose, waypoints):

    best_index = 0
    best_distance = get_distance_between_points(pose.position, waypoints[0].pose.pose.position)

    for index, waypoint in enumerate(waypoints):

        distance = get_distance_between_points(pose.position, waypoint.pose.pose.position)

        if distance < best_distance:

            best_index = index
            best_distance = distance

    return best_index
