#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 80 # Number of waypoints we will publish. You can change this number
ONE_MPH = 0.44704


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.pose = None
        self.traffic_waypoint = None
        self.current_linear_velocity = 0.0
        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') / 2.2369

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if (self.waypoints is None) or (self.pose is None):
                continue

            near_dist  = 99999
            near_index = None
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
            for i in range(len(self.waypoints.waypoints)):
                dist = dl(self.pose.pose.position, self.waypoints.waypoints[i].pose.pose.position)
                if dist < near_dist:
                    near_dist  = dist
                    near_index = i
            near_index = (near_index + 1) % len(self.waypoints.waypoints)

            lane = Lane()
            lane.waypoints = []

            v = self.current_linear_velocity
            v = max(1.1 * v, v + ONE_MPH)
            v = min(v, self.speed_limit - ONE_MPH)

            #for i in range(LOOKAHEAD_WPS):
                #index  = (i + near_index    ) % len(self.waypoints.waypoints)
                #lane.waypoints.append(self.waypoints.waypoints[index])
                #lane.waypoints[i].pose.header.seq = index

            for i in range(LOOKAHEAD_WPS):
                index  = (i + near_index     ) % len(self.waypoints.waypoints)
                index2 = (i + near_index +  5) % len(self.waypoints.waypoints)
                index3 = (i + near_index + 15) % len(self.waypoints.waypoints)
                lane.waypoints.append(self.waypoints.waypoints[index])
                self.set_waypoint_velocity(lane.waypoints, i, v)

                dist = 0.0
                if i < LOOKAHEAD_WPS-1:
                    dist = self.distance(self.waypoints.waypoints, index, index2)

                if self.traffic_waypoint == None or self.traffic_waypoint == -1:
                    v_sq = v * v + 2. * dist
                    v = math.sqrt(max(0., v_sq))
                    v = min(v, self.speed_limit - ONE_MPH)
                else:
                    if index2 == self.traffic_waypoint:
                        v = 0
                    elif index3 < self.traffic_waypoint:
                        v_sq = v * v + 2. * dist
                        v = math.sqrt(max(0., v_sq))
                        v = min(v, self.speed_limit - ONE_MPH)
                    else:
                        #v_sq = 0.6 * v * v - 2. * dist
                        v = 0
                        v = math.sqrt(max(0., v_sq))
                        v = min(0, v)

            self.final_waypoints_pub.publish(lane)
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data
        rospy.logwarn("way : %d", self.traffic_waypoint)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear.x

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
