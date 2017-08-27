#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import tf.transformations
import cv2
from traffic_light_config import config
import tf_helper
import numpy as np


STATE_COUNT_THRESHOLD = 3


class TLDetector(object):

    def __init__(self):
        rospy.init_node('tl_detector')

        self.car_pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/camera/image_raw', Image, self.image_cb, queue_size=1)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.image_pub = rospy.Publisher('/camera/my_image', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.last_traffic_light_state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.last_reported_traffic_light_id = None
        self.last_reported_traffic_light_time = None

        self.traffic_lights = None
        self.image = None

        rospy.spin()

    def pose_cb(self, msg):
        self.car_pose = msg.pose

        arguments = [self.traffic_lights, self.car_pose, self.waypoints, self.image]
        are_arguments_available = all([x is not None for x in arguments])

        if are_arguments_available:

            # Get closest traffic light
            traffic_light = tf_helper.get_closest_traffic_light_ahead_of_car(
                self.traffic_lights, self.car_pose.position, self.waypoints)

            x, y = self.project_to_image_plane(traffic_light.pose.pose.position, self.car_pose)

            image_width = config.camera_info.image_width
            image_height = config.camera_info.image_height

            # Only try to classify image if traffic light is within it
            if 0 < x < image_width and 0 < y < image_height:

                self.camera_image = self.image
                self.camera_image.encoding = "rgb8"

                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

                traffic_light_state = self.light_classifier.get_classification(cv_image)

                cv2.circle(cv_image, (x, y), radius=50, color=(255, 0, 0), thickness=12)
                marked_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                self.image_pub.publish(marked_image)

                if traffic_light_state == TrafficLight.RED:

                    traffic_light_waypoint_index = tf_helper.get_closest_waypoint_index(
                        traffic_light.pose.pose.position, self.waypoints)

                    self.upcoming_red_light_pub.publish(Int32(traffic_light_waypoint_index))
                    rospy.logwarn("Publishing red light at waypoint: {}".format(traffic_light_waypoint_index))

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.traffic_lights = msg.lights

    def image_cb(self, msg):
        self.image = msg

    def project_to_image_plane(self, point_in_world, car_pose):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = config.camera_info.focal_length_x
        fy = config.camera_info.focal_length_y

        image_width = config.camera_info.image_width
        image_height = config.camera_info.image_height

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        # rospy.logwarn("Transform shift is: {}".format(trans))
        # rospy.logwarn("Rotation is: {}".format(rot))

        # TODO Use transform and rotation to calculate 2D position of light in image
        world_coordinates_point = np.array(
            [point_in_world.x, point_in_world.y, point_in_world.z], dtype=np.float32).reshape(3, 1)

        car_position = np.array([car_pose.position.x, car_pose.position.y, car_pose.position.z], dtype=np.float32).reshape(3, 1)
        camera_offset = np.array([1.0, 0, 1.2], dtype=np.float32).reshape(3, 1)
        # translation_vector = np.array(trans, dtype=np.float32).reshape(3, 1)
        translation_vector = car_position + camera_offset

        # Move point to camera origin
        world_coordinates_point_shifted_to_camera_coordinates = world_coordinates_point - translation_vector

        homogenous_vector = np.ones(shape=(4, 1), dtype=np.float32)
        homogenous_vector[:3] = world_coordinates_point_shifted_to_camera_coordinates

        quaternion = np.array([
            car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w],
            dtype=np.float32)

        euler_angles = tf.transformations.euler_from_quaternion(quaternion)
        rotation_matrix = tf.transformations.euler_matrix(*euler_angles)

        point_in_camera_coordinates = np.dot(rotation_matrix, homogenous_vector)

        x = (fx * point_in_camera_coordinates[0] * point_in_camera_coordinates[2]) + (image_width / 2)
        y = (fy * point_in_camera_coordinates[1] * point_in_camera_coordinates[2]) + (image_height / 2)

        return int(x), int(y)


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

