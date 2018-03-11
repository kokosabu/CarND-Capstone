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
        self.velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))

        #rospy.spin()
        self.loop()

    def loop(self):
        #rate = rospy.Rate(10)
        rate = rospy.Rate(2)

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

            #"""
            last_index = near_index + LOOKAHEAD_WPS
            l = False
            if last_index > len(self.waypoints.waypoints):
                waypoints = self.waypoints.waypoints[near_index:]
                waypoints += self.waypoints.waypoints[:last_index - len(self.waypoints.waypoints)]
                l = True
            else:
                waypoints = self.waypoints.waypoints[near_index:last_index]

            if self.traffic_waypoint == None or self.traffic_waypoint == -1:
                brake_wp = -1
                brake = True
            elif l and self.traffic_waypoint < near_index:
                brake_wp = self.traffic_waypoint - near_index + len(self.waypoints.waypoints)
                brake = brake_wp <= 30
            elif self.traffic_waypoint < near_index:
                brake_wp = -1
                brake = True
            else:
                brake_wp = self.traffic_waypoint - near_index
                brake = brake_wp <= 30

            #rospy.logwarn("wp : %d, brake : %d\n", brake_wp, brake)

            for i in range(len(waypoints)):
                #v_sq = v * v + 2. * dist
                #v = math.sqrt(max(0., v_sq))
                #v = min(v, self.speed_limit - ONE_MPH)
                #self.set_waypoint_velocity(waypoints, i, v)
                set_v = min(self.current_linear_velocity + (i+1) * 2, self.velocity)
                self.set_waypoint_velocity(waypoints, i, set_v)

            if brake:
                self.set_waypoint_velocity(waypoints, brake_wp, 0.0)
                #self.set_waypoint_velocity(waypoints, brake_wp, -0.5)
                for i in range(0, brake_wp+1):
                    dist = self.distance(waypoints, brake_wp, i)
                    set_v = math.sqrt(2 * 1 * dist)
                    if set_v < 1:
                        set_v = 0.5
                    self.set_waypoint_velocity(waypoints, i, min(set_v,
                        waypoints[i].twist.twist.linear.x))

            lane.waypoints = waypoints
            self.final_waypoints_pub.publish(lane)

            #for i in range(LOOKAHEAD_WPS):
                #index  = (i + near_index    ) % len(self.waypoints.waypoints)
                #lane.waypoints.append(self.waypoints.waypoints[index])
                #lane.waypoints[i].pose.header.seq = index
            

            """

            count1 = 0
            count2 = 0
            count3 = 0
            count4 = 0
            count5 = 0
            for i in range(LOOKAHEAD_WPS):
                index  = (i + near_index     ) % len(self.waypoints.waypoints)
                index2 = (i + near_index +  5) % len(self.waypoints.waypoints)
                index3 = (i + near_index + 15) % len(self.waypoints.waypoints)
                lane.waypoints.append(self.waypoints.waypoints[index])
                self.set_waypoint_velocity(lane.waypoints, i, v)

                dist = 0.0
                if i < LOOKAHEAD_WPS-1:
                    dist = self.distance(self.waypoints.waypoints, index, index2)
                    count1 += 1

                if self.traffic_waypoint == None or self.traffic_waypoint == -1:
                    v_sq = v * v + 2. * dist
                    v = math.sqrt(max(0., v_sq))
                    v = min(v, self.speed_limit - ONE_MPH)
                    v = max(v, v + ONE_MPH)
                    v = 0
                    count2 += 1
                else:
                    if index2 == self.traffic_waypoint:
                        v = 0
                        count3 += 1
                    elif index3 < self.traffic_waypoint:
                        #v_sq = v * v + 2. * dist
                        v = 0
                        v = math.sqrt(max(0., v_sq))
                        v = min(v, self.speed_limit - ONE_MPH)
                        v = max(v, v + ONE_MPH)
                        count4 += 1
                    else:
                        #v_sq = 0.6 * v * v - 2. * dist
                        v = 0
                        v = math.sqrt(max(0., v_sq))
                        v = min(v, self.speed_limit - ONE_MPH)
                        v = max(v, v + ONE_MPH)
                        count5 += 1

                rospy.logwarn("i = %d, v = %f, idx1 = %d, idx2 = %d, idx3 = %d",
                        i, v, index, index2, index3)

            rospy.logwarn("%d %d %d %d %d",
                    count1, count2, count3, count4, count5)
            """
            self.final_waypoints_pub.publish(lane)
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data
        #rospy.logwarn("way : %d", self.traffic_waypoint)

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

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.0) / (60.0 * 60.0)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
