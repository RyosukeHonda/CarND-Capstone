"""
Utilities for traffic light module
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


def get_closest_waypoint_index(position, waypoints):
    """
    Given a pose and waypoints list, return index of waypoint closest to pose
    :param position: geometry_msgs.msgs.Position instance
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: integer index
    """

    best_index = 0
    best_distance = get_distance_between_points(position, waypoints[0].pose.pose.position)

    for index, waypoint in enumerate(waypoints):

        distance = get_distance_between_points(position, waypoint.pose.pose.position)

        if distance < best_distance:

            best_index = index
            best_distance = distance

    return best_index


def get_closest_traffic_light(traffic_lights, car_position, waypoints):
    """
    Given list of traffic lights, car position and waypoints, return closest traffic light
    ahead of the car
    :param traffic_lights: list of styx_msgs.msg.TrafficLight instances
    :param car_position: geometry_msgs.msgs.Pose instance
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: styx_msgs.msg.TrafficLight instance
    """

    car_waypoint_index = get_closest_waypoint_index(car_position, waypoints)
    traffic_lights_waypoint_indices = []

    for traffic_light in traffic_lights:

        index = get_closest_waypoint_index(traffic_light.pose.pose.position, waypoints)
        traffic_lights_waypoint_indices.append(index)

    differences = np.array(traffic_lights_waypoint_indices) - car_waypoint_index
    index = np.argmin(np.abs(differences))
    return traffic_lights[index]