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
POSE_LIMIT = 5  # Number to decide After how many pose function callbacks to publish
                # Change this number to see the latency problem.   


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.loginfo("\n ** WaypointUpdarter class Init is called **")
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.wpts = Lane()   # base waypoints variable
        self.fwpts = Lane()  # final waypoints varialble
        self.pose_calls = 0  # counter for the number of pose_cb callbacks.
        rospy.spin()
    
    def pose_cb(self, msg):
        # TODO: Implement
        # calucalate how many times the call back function is called
        if(self.pose_calls < POSE_LIMIT) : 
            self.pose_calls +=1
        else:
            self.pose_calls = 0    
        
        # This condition is only used to reduce the computation frequency
        # of the call back function
        if(self.pose_calls == POSE_LIMIT):
            # logging the position of the car every two seconds
            rospy.loginfo_throttle(2, "\n Car Position: "+str(msg.pose.position ))

            # Macro to compute the distance copied from distance function
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

            #Minimum distance equal to the first value
            distM = dl(self.wpts.waypoints[0].pose.pose.position,msg.pose.position)  
            
            wptP = 0  # closest waypoint index
            # finding the closest waypoint to the car
            for i in range(1,len( self.wpts.waypoints)):
                # Computing the distance
                dist = dl(self.wpts.waypoints[i].pose.pose.position,msg.pose.position)
                if(dist<distM):
                    wptP = i;
                    distM = dist;
            # logging the closest waypoint index
            rospy.loginfo_throttle(2,"\n closest waypoint index: "+str(wptP))
   
            # constructing the final waypoints
            i = 0
            self.fwpts.waypoints = []
            # Make sure not to exceed the max index of base waypoints
            if((wptP+LOOKAHEAD_WPS+1)<len( self.wpts.waypoints)):
                # Copy the final waypoints 
                for j in range(wptP+1,wptP+LOOKAHEAD_WPS+1):
                    self.fwpts.waypoints.append(self.wpts.waypoints[j])
                    self.fwpts.waypoints[i].twist.twist.linear.x = self.wpts.waypoints[j].twist.twist.linear.x
                    i+=1
            else:
                # Copy the first part of final waypoints till the end of the map
                for j in range(wptP+1,len(self.wpts.waypoints)-1):
                    self.fwpts.waypoints.append(self.wpts.waypoints[j])
                    self.fwpts.waypoints[i].twist.twist.linear.x = self.wpts.waypoints[j].twist.twist.linear.x
                    i+=1
                # Copy the second part at the begining of the map
                for j in range(0,LOOKAHEAD_WPS-(len(self.wpts.waypoints)-wptP)):
                    self.fwpts.waypoints.append(self.wpts.waypoints[j])
                    self.fwpts.waypoints[i].twist.twist.linear.x = self.wpts.waypoints[j].twist.twist.linear.x
                    i+=1
             
            self.final_waypoints_pub.publish(self.fwpts)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.wpts = waypoints;
        rospy.loginfo("\n ** Base waypoints copied ** ")
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
