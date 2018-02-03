#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

	#Part relevant for waypoint_updater.py full solution with traffic and obstacles
	#rospy.Subscriber('/traffic_waypoint', Waypoint, self.traffic_cb)
	#rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.wpts = Lane()   # base waypoints variable
        self.fwpts = Lane()  # final waypoints varialble
        
        rospy.spin()
    
    def pose_cb(self, msg):
        # TODO: Implement

        # Macro to compute the distance copied from distance function
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
   
        distM = 1000 # High value Minimum distance
        wptP = 0     # closest waypoint index
        # finding the closest waypoint to the car
        for i in range(len( self.wpts.waypoints)):
            # Computing the distance
            dist = dl(self.wpts.waypoints[i].pose.pose.position,msg.pose.position)
            if(dist<distM):
                wptP = i;
                distM = dist;
        

        # constructing the final waypoints
        i = 0
        self.fwpts.waypoints = []
        for j in range(wptP,wptP+LOOKAHEAD_WPS):
            self.fwpts.waypoints.append(self.wpts.waypoints[j])
            self.fwpts.waypoints[i].twist.twist.linear.x = self.wpts.waypoints[j].twist.twist.linear.x
            i+=1

        # Publishing the final waypoints
        rate = rospy.Rate(50) # 50hz

        # do we need the while loop in this case ??
        while not rospy.is_shutdown():
            self.final_waypoints_pub.publish(self.fwpts)
            rate.sleep()
            
        
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        print ("*********** waypoints list call *******************")
        self.wpts = waypoints;
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
