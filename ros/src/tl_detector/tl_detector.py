#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Waypoint
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
import tl_ssd_detector as ssd


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
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.sight_distance = 100

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
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
            light_wp = light_wp 
            if state == TrafficLight.RED else -1
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
        # return -1 if none exists
        if self.waypoints is None:
            return -1
        minDist = 1e6
        minIdx = -1      
        px = pose.position.x
        py = pose.position.y
        #rospy.loginfo("len(wp): %d", len(self.waypoints))
        for idx in range(len(self.waypoints)):
            waypoint = self.waypoints[idx].pose.pose;
            wpx = waypoint.position.x
            wpy = waypoint.position.y
            dist = math.sqrt((px - wpx)**2 + (py - wpy)**2)
            if (dist < minDist): 
                minIdx = idx    # index of the closest waypoint
                minDist = dist     # and distance of the closest waypoint
        return minIdx


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
       
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        cv2.imwrite("screenshot.jpg", cv_image)
        image = ssd.Image.open('screenshot.jpg')
        imagecropped = ssd.pipeline_ssd(image)
        #Get classification
        if imagecropped is None:
            return 4 # none = id #4
        else:
            return self.light_classifier.get_classification(imagecropped)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_mindist_stop_wp = None
        light_dist = 1e6       

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN
        
        #find the closest traffic light
        for sl_pos in stop_line_positions:
            light_stop_pose = Pose()
            light_stop_pose.position.x = sl_pos[0]
            light_stop_pose.position.y = sl_pos[1]
            light_stop_wp = self.get_closest_waypoint(light_stop_pose)     #get the wp closest to each light_position
            if light_stop_wp >= car_position :    #it found a waypoint close to the traffic light and ahead of the car
                if light_mindist_stop_wp is None:    #check if this is the first light we process
                    light_mindist_stop_wp = light_stop_wp
                    light = light_stop_pose
                elif light_stop_wp < light_mindist_stop_wp:
                    light_mindist_stop_wp = light_stop_wp    #if  light_wp ahead of the car position allocate closer value
                    light = light_stop_pose

        if ((car_position is not None) and (light_mindist_stop_wp is not None)):
            light_dist  = abs(car_position - light_mindist_stop_wp)

        if light and light_dist  < self.sight_distance: # check traffic light is in the range of a predifined number of waypoints
            state = self.get_light_state(light)
            return light_mindist_stop_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
