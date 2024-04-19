#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, cos, sin
from dynamic_reconfigure.server import Server
from scout_scripts.cfg import OmniMoverConfig
from visualization_msgs.msg import Marker, MarkerArray

class OmniMoverWithWaypoints:
    def __init__(self):
        rospy.init_node('omni_mover_with_waypoints')
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.marker_pub = rospy.Publisher('waypoint_markers', MarkerArray, queue_size=10)

        self.waypoints = [(0.0, 0.0) for _ in range(5)]  # Initialize with 5 waypoints
        self.current_waypoint_index = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.move_speed = 1.16
        self.waypoint_tolerance = 0.3
        self.is_ready_to_move = False

        self.srv = Server(OmniMoverConfig, self.config_callback)
        self.publish_markers()

    def config_callback(self, config, level):
        for i in range(5):
            self.waypoints[i] = (getattr(config, "waypoint{}_x".format(i)), getattr(config, "waypoint{}_y".format(i)))
        rospy.loginfo("Updated waypoints: %s", self.waypoints)
        self.publish_markers()
        return config

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def wait_for_key(self):
        raw_input("Press Enter to start moving to waypoints...")
        self.is_ready_to_move = True

    def move_to_waypoints(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.is_ready_to_move and self.current_waypoint_index < len(self.waypoints):
                self.move_to_next_waypoint()
            rate.sleep()

    def move_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        distance = self.distance_to_waypoint(target_x, target_y)

        if distance < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.waypoints):
                self.stop_robot()
                return

        angle = atan2(target_y - self.current_y, target_x - self.current_x)
        twist = Twist()
        twist.linear.x = self.move_speed * cos(angle)
        twist.linear.y = self.move_speed * sin(angle)
        self.vel_pub.publish(twist)

    def distance_to_waypoint(self, waypoint_x, waypoint_y):
        return sqrt((waypoint_x - self.current_x)**2 + (waypoint_y - self.current_y)**2)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    mover = OmniMoverWithWaypoints()
    mover.wait_for_key()
    mover.move_to_waypoints()
