"""
Utilities for traffic light module
"""

import numpy as np
import cv2


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


def get_closest_traffic_light_ahead_of_car(traffic_lights, car_position, waypoints):
    """
    Given list of traffic lights, car position and waypoints, return closest traffic light
    ahead of the car
    :param traffic_lights: list of styx_msgs.msg.TrafficLight instances
    :param car_position: geometry_msgs.msgs.Pose instance
    :param waypoints: list of styx_msgs.msg.Waypoint instances
    :return: styx_msgs.msg.TrafficLight instance
    """

    car_waypoint_index = get_closest_waypoint_index(car_position, waypoints)

    lights_waypoints_indices = []

    for traffic_light_index, traffic_light in enumerate(traffic_lights):

        index = get_closest_waypoint_index(traffic_light.pose.pose.position, waypoints)

        if index > car_waypoint_index:
            lights_waypoints_indices.append((traffic_light_index, index - car_waypoint_index))

    sorted_traffic_lights_waypoint_indices = sorted(lights_waypoints_indices, key=lambda x: x[1])
    light_index = sorted_traffic_lights_waypoint_indices[0][0]
    return traffic_lights[light_index]


def draw_marker(image, x, y):
    """
    Draw a simple marker around x, y coordinates in an image
    :param image: numpy array
    :param x: integer
    :param y: integer
    """

