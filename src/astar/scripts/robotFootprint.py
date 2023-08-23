import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np

# ROS Initialization
rospy.init_node('robot_simulation')

# Define robot parameters
robot_radius = 0.5  # Radius of the robot's circular footprint in meters

# Initialize TF2 buffer and listener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Initialize robot position
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
# Initialize TF2 buffer and broadcaster
# tf_buffer = tf2_ros.Buffer()
tf_broadcaster = tf2_ros.TransformBroadcaster()

# Callback function for odometry
def odometry_callback(odom_msg):
    global robot_x, robot_y, robot_theta

    robot_x = odom_msg.pose.pose.position.x
    robot_y = odom_msg.pose.pose.position.y

    # Convert quaternion to euler angles for robot orientation
    quaternion = (
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    )
    euler_angles = euler_from_quaternion(quaternion)
    robot_theta = euler_angles[2]

# ROS subscribers
odom_sub = rospy.Subscriber('odom', Odometry, odometry_callback)

# ROS publisher for the robot's footprint polygon
polygon_pub = rospy.Publisher('robot_footprint_polygon', PolygonStamped, queue_size=1)

rate = rospy.Rate(10)  # Update rate (10 Hz in this example)
while not rospy.is_shutdown():
    # transform_stamped = geometry_msgs.msg.TransformStamped()
    # transform_stamped.header.stamp = rospy.Time.now()
    # transform_stamped.header.frame_id = 'map'
    # transform_stamped.child_frame_id = 'base_link'
    # transform_stamped.transform.translation.x = robot_x
    # transform_stamped.transform.translation.y = robot_y
    # transform_stamped.transform.translation.z = 0.0
    # transform_stamped.transform.rotation.x = 0.0
    # transform_stamped.transform.rotation.y = 0.0
    # transform_stamped.transform.rotation.z = np.sin(robot_theta / 2.0)
    # transform_stamped.transform.rotation.w = np.cos(robot_theta / 2.0)

    # tf_broadcaster.sendTransform(transform_stamped)
    try:
        # Lookup the transform between the 'map' and 'base_link' frames
        transform = tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("No tf")
        continue
    

    # Update robot position from the TF transform
    robot_x = transform.transform.translation.x
    robot_y = transform.transform.translation.y

    # Convert quaternion to euler angles for robot orientation
    euler_angles = euler_from_quaternion([
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    ])
    robot_theta = euler_angles[2]

    # Create and publish the robot's footprint polygon
    polygon_msg = PolygonStamped()
    polygon_msg.header.stamp = rospy.Time.now()
    polygon_msg.header.frame_id = 'map'
    polygon_msg.polygon.points = []

    num_points = 36  # Number of points to approximate the circle
    for i in range(num_points):
        angle = i * (2 * np.pi / num_points)
        x = robot_x + robot_radius * np.cos(angle)
        y = robot_y + robot_radius * np.sin(angle)
        polygon_msg.polygon.points.append(Point32(x, y, 0.0))

    polygon_pub.publish(polygon_msg)

    rate.sleep()
