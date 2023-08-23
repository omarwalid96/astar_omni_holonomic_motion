import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
# ROS Initialization
rospy.init_node('omni_drive_controller')
tf_broadcaster = TransformBroadcaster()
map_to_odom_broadcaster = TransformBroadcaster()

# Define robot parameters
wheel_radius = 0.042  # Radius of the omni wheels in meters
wheel_base = 0.2     # Distance between the wheels in meters
angular_velocity = math.pi / 6  # Angular velocity for each wheel (assuming the same for all wheels)

# Initialize robot position and orientation
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
current_time=rospy.Time.now()
last_callback_time=rospy.Time.now()
class OdometryPublisher:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = 0.042
        self.expected_distance = 0.0
        self.actual_distance = 0.0

    def update(self, linear_velocity_x, linear_velocity_y, angular_velocity, delta_t):
        delta_theta = angular_velocity * delta_t
        avg_theta = self.theta + delta_theta 

        delta_x = (linear_velocity_x * math.cos(avg_theta) - linear_velocity_y * math.sin(avg_theta)) * delta_t
        delta_y = (linear_velocity_x * math.sin(avg_theta) + linear_velocity_y * math.cos(avg_theta)) * delta_t

        self.expected_distance += math.sqrt(linear_velocity_x ** 2 + linear_velocity_y ** 2) * delta_t
        self.actual_distance += self.wheel_radius * abs(angular_velocity) * delta_t

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def get_odometry(self):
        return self.x, self.y, self.theta

# def calculate_odometry_from_velocities(Vx, Vy, Vw, delta_t):
#     delta_x = Vx * delta_t
#     delta_y = Vy * delta_t
#     delta_theta = Vw * delta_t

#     return delta_x, delta_y, delta_theta

odometry_publisher = OdometryPublisher()

def publishTwist():
    # v1= -0.5*linear_x + (math.sqrt(3)/2)*linear_y + l*angular_w
    # v2= -0.5*linear_x - (math.sqrt(3)/2)*linear_y + l*angular_w
    # v3= linear_x + l*angular_w


    # M1 = (-2.6908 * linear_x) + (-0.8612 * linear_y) + (2.6908 * angular_w)
    # M2 = (4.3816 * linear_x) + (3.1246 * linear_y) + (-4.3816 * angular_w)
    # M3 = (-1.6908 * linear_x) + (-2.2634 * linear_y) + (2.6908 * angular_w)
    motor1= -0.26908
    motor2=0.43815999999999994
    motor3=-0.16907999999999998
    Vx=motor1*math.cos(240)+motor2*math.cos(120)+motor3*math.cos(0)
    Vy=motor1*math.sin(240)+motor2*math.sin(120)+motor3*math.sin(0)
    Vw=motor1+motor2+motor3
    vel=Twist()
    vel.linear.x=Vx
    vel.linear.y=Vy
    vel.angular.z=Vw
    twist_pub.publish(vel)
# Callback function for twist commands
def twist_callback(twist_msg):
    global robot_x, robot_y, robot_theta,last_callback_time,current_time
    current_time = rospy.Time.now()
    delta_t = (current_time - last_callback_time).to_sec()
    Vx=twist_msg.linear.x
    Vy=twist_msg.linear.y
    Vw=twist_msg.angular.z*wheel_radius
    
    # delta_x, delta_y, delta_theta = calculate_odometry_from_velocities(Vx, Vy, Vw, delta_t)
    odometry_publisher.update(Vx, Vy, Vw, delta_t)
    robot_x,robot_y,robot_theta= odometry_publisher.get_odometry()

    # Publish odometry position
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'
    odom_msg.pose.pose.position.x = robot_x
    odom_msg.pose.pose.position.y = robot_y
    quaternion = quaternion_from_euler(0.0, 0.0, robot_theta)
    odom_msg.pose.pose.orientation.x = quaternion[0]
    odom_msg.pose.pose.orientation.y = quaternion[1]
    odom_msg.pose.pose.orientation.z = quaternion[2]
    odom_msg.pose.pose.orientation.w = quaternion[3]
    odom_msg.twist.twist.linear.x = Vx
    odom_msg.twist.twist.linear.y = Vy
    odom_msg.twist.twist.angular.z = Vw

    odom_pub.publish(odom_msg)
    # Publish tf transform
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = 'odom'
    transform_stamped.child_frame_id = 'base_link'
    transform_stamped.transform.translation.x = robot_x
    transform_stamped.transform.translation.y = robot_y
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]
    tf_broadcaster.sendTransform(transform_stamped)

    # Publish map to odom transform
    map_to_odom = TransformStamped()
    map_to_odom.header.stamp = rospy.Time.now()
    map_to_odom.header.frame_id = 'map'
    map_to_odom.child_frame_id = 'odom'
    map_to_odom.transform.translation.x = robot_x
    map_to_odom.transform.translation.y = robot_y
    map_to_odom.transform.rotation.x = 0.0
    map_to_odom.transform.rotation.y = 0.0
    map_to_odom.transform.rotation.z = 0.0
    map_to_odom.transform.rotation.w = 1.0
    map_to_odom_broadcaster.sendTransform(map_to_odom)
    last_callback_time = current_time

# ROS subscribers and publishers
twist_sub = rospy.Subscriber('cmd_vel', Twist, twist_callback)
odom_pub = rospy.Publisher('odom_test', Odometry, queue_size=1)
twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(50)  # Update rate (10 Hz in this example)
while not rospy.is_shutdown():
    # publishTwist()
    rate.sleep()
