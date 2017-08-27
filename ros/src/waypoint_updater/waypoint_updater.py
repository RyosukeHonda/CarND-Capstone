#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import std_msgs.msg
import os

import waypoints_helper
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

miles_per_hour_to_metres_per_second = 0.44704


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', std_msgs.msg.Int32, self.traffic_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.last_base_waypoints_lane = None
        self.upcoming_traffic_light_waypoint_id = None
        self.last_upcoming_traffic_light_message_time = None

        rospy.spin()

    def pose_cb(self, msg):

        # TODO: Implement
        if self.last_base_waypoints_lane is not None:

            pose = msg.pose
            base_waypoints = self.last_base_waypoints_lane.waypoints

            lane = Lane()
            lane.header.stamp = rospy.Time.now()

            car_waypoint_index = waypoints_helper.get_closest_waypoint_index(pose, base_waypoints)
            lane.waypoints = waypoints_helper.get_sublist(base_waypoints, car_waypoint_index, LOOKAHEAD_WPS)

            # lane.waypoints = waypoints_helper.get_smoothed_out_waypoints(lane.waypoints)

            for index in range(len(lane.waypoints)):

                lane.waypoints[index].twist.twist.linear.x = 15.0 * miles_per_hour_to_metres_per_second

            is_red_light_ahed = self.upcoming_traffic_light_waypoint_id is not None and \
                self.upcoming_traffic_light_waypoint_id > car_waypoint_index

            if is_red_light_ahed and not self.is_traffic_light_message_stale():

                traffic_light_id = self.upcoming_traffic_light_waypoint_id - car_waypoint_index

                for index in range(len(lane.waypoints)):

                    lane.waypoints[index].twist.twist.linear.x = -100

            self.final_waypoints_pub.publish(lane)

            path = "/home/student/Downloads/final_waypoints.txt"

            if not os.path.exists(path):

                waypoints_helper.save_waypoints(lane.waypoints, path)

    def base_waypoints_cb(self, lane):
        # TODO: Implement
        self.last_base_waypoints_lane = lane

        # if not os.path.exists(path):
        #     waypoints_helper.save_waypoints(lane.waypoints, path)

    def traffic_cb(self, msg):

        # TODO: Callback for /traffic_waypoint message. Implement
        self.upcoming_traffic_light_waypoint_id = msg.data
        self.last_upcoming_traffic_light_message_time = rospy.get_rostime()

    def is_traffic_light_message_stale(self):

        ros_duration = rospy.get_rostime() - self.last_upcoming_traffic_light_message_time
        duration_in_seconds = ros_duration.secs + (1e-9 * ros_duration.nsecs)
        return duration_in_seconds > 0.5

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
