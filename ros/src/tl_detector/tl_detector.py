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
import cv2
from traffic_light_config import config

import math
import numpy as np


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
	self.image_pub = rospy.Publisher('/camera/my_image', Image,queue_size = 1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
	
	

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
	msg.encoding = "rgb8"
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
	

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
	min_index = 0
	min_dist =9999999.9
	for index,waypoint in enumerate(self.waypoints.waypoints[:]):
	    dist = self.get_distance(pose,waypoint)
	    if(dist < min_dist):
		min_dist = dist
		min_index = index
        return min_index

    def get_distance(self,pose,waypoint):
	dist = math.sqrt((pose.position.x - waypoint.pose.pose.position.x)**2 +
			  (pose.position.y - waypoint.pose.pose.position.y)**2)
	return dist
	


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """
	#rospy.logwarn("TRAFFIC LIGHT DETECTED!")
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

        #TODO Use tranform and rotation to calculate 2D position of light in image
	
	#4x4 homogenerous rotation matrix
	quaternion_matrix = tf.transformations.quaternion_matrix(rot)

	camera_intrinsic = np.array([[fx,0,image_width/2.0],
			             [0,fy,image_height/2.0],
				     [0,0,1]])
	#3x3 rotation matrix
	quaternion_matrix = quaternion_matrix[:3,:3]
	#transition
	transition = np.array([[trans[0]],[trans[1]],[trans[2]]])
	#rotation-transition matrix
	rot_trans = np.hstack((quaternion_matrix,transition))
	x,y,z =np.dot(np.dot(camera_intrinsic,rot_trans),np.array		  ([point_in_world.x,point_in_world.y,point_in_world.z,1.0]))
	
	x = x//z
	y = y//z

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
	
        x, y = self.project_to_image_plane(light.pose.pose.position)
	rospy.logwarn("X:{0}  Y:{1}".format(x,y))
	#cv_image2 = cv_image[y-200:y+200,x-200:x+200]
	#publish image 
	cv2.circle(cv_image,(int(y),int(x)), radius = 5, color=(255,0,0),thickness = 12)
	marked_image = self.bridge.cv2_to_imgmsg(cv_image,encoding="bgr8")
	self.image_pub.publish(marked_image)
	

	#TODO use light location to zoom in on traffic light in image
	#crop the original image(Need to tune the parameters)
	
	
	
	#Test use
	#cv2.imshow("Image window",cv_image2)
	#cv2.waitKey(3)

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = config.light_positions
	
	data =[self.pose,self.waypoints]
	is_all_available = all([x is not None for x in data])

        if is_all_available:
	    #get the waypoint index that is the nearest from the car position
            car_position_index = self.get_closest_waypoint(self.pose.pose)
	    

        #TODO find the closest visible traffic light (if one exists)
	    #Nearest waypoint from the car position
	    nearest_waypoint =self.waypoints.waypoints[car_position_index]
	    min_dist =99999999
	    min_index = 999
	    
	    for index, light_position in enumerate(light_positions):
	        dist = math.sqrt((light_position[0] - nearest_waypoint.pose.pose.position.x)**2 +
			     (light_position[1] - nearest_waypoint.pose.pose.position.y)**2)
		
	        if dist < min_dist:
		    min_dist = dist	
	            min_index = index
		    # get nearest waypoint from the traffic light
		    light_index_in_basewp = self.nearest_waypoint_from_light(light_position,self.waypoints.waypoints)
		
		    
		traffic_light_ahead = 100
		#Traffic light should be ahead of the car
 	        if min_dist < traffic_light_ahead and car_position_index < light_index_in_basewp: #Need to Fix(Distance to the Traffic Light)
		
	            light = TrafficLight()  
	            light.pose.pose.position.x = light_positions[min_index][0]
	            light.pose.pose.position.y = light_positions[min_index][1]
		    light.pose.pose.position.z = 5.837643
	        
        if light:
            state = self.get_light_state(light)
	    state = self.nearest_light_state(light, self.lights) #This is for test use(use ground truth data)
	    light_wp = min_index # Nearest traffic light waypoint's index
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN


    def nearest_light_state(self,light,gt_lights):
	#Args 
	#light: the nearest light from the vehicle
	#gt_lights: lights information given by TrafficLightArray(Ground Truth)
	
	# This function is for test use.(Without using transformation and classifier)
	
	#Returns
	#state: the state of the signal
	min_dist = 9999999
	min_index = 999
	for index,gt_light in enumerate(gt_lights):
	    dist = math.sqrt((light.pose.pose.position.x - gt_light.pose.pose.position.x)**2 +
			     (light.pose.pose.position.y - gt_light.pose.pose.position.y)**2)
	    if(dist < min_dist):
		min_dist = dist
		min_index = index
	return gt_lights[min_index].state

    def nearest_waypoint_from_light(self,light_position,waypoints):
	min_dist = 9999999
	min_index = 999
	for index, waypoint in enumerate(waypoints):
	    dist = math.sqrt((light_position[0]-waypoint.pose.pose.position.x)**2+(light_position[1]-waypoint.pose.pose.position.y)**2)
	    if dist < min_dist:
		min_dist = dist
		min_index = index
	return min_index
	

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
