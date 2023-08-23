#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header

def publish_goal_marker():
    rospy.init_node('goal_marker_publisher')

    # Create a goal marker (2D circle)
    goal_marker = Marker()
    goal_marker.header = Header()
    goal_marker.header.frame_id = 'map'
    goal_marker.id = 0
    goal_marker.type = Marker.CYLINDER
    goal_marker.action = Marker.ADD
    goal_marker.pose.position = Point(10.0, 10.0, 0.0)  # Change this to your desired goal position
    goal_marker.pose.orientation.w = 1.0
    goal_marker.scale.x = 0.5  # Diameter of the circle
    goal_marker.scale.y = 0.5  # Diameter of the circle
    goal_marker.scale.z = 0.001  # Very small value for a 2D circle

    goal_marker.color.a = 1.0
    goal_marker.color.r = 0.0
    goal_marker.color.g = 1.0
    goal_marker.color.b = 0.0

    marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)

    rate = rospy.Rate(1)  # Publish marker once per second
    while not rospy.is_shutdown():
        goal_marker.header.stamp = rospy.Time.now()
        marker_pub.publish(goal_marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal_marker()
    except rospy.ROSInterruptException:
        pass
