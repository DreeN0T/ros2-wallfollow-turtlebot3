# Python math library
import math

# ROS client library for Python
import rclpy

# Enables pauses in the execution of code
from time import sleep

# Used to create nodes
from rclpy.node import Node

# Enables the use of the string message type
from std_msgs.msg import String

# Twist is linear and angular velocity
from geometry_msgs.msg import Twist

# Handles LaserScan messages to sense distance to obstacles (i.e. walls)
from sensor_msgs.msg import LaserScan

# Handle Pose messages
from geometry_msgs.msg import Pose

# Handle float64 arrays
from std_msgs.msg import Float64MultiArray

# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data

# Scientific computing library
import numpy as np


class Controller(Node):


    def __init__(self):
        super().__init__('Controller')

        # Create a subscriber

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)


        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)


        self.left_dist = 999999.9 # Left
        self.leftfront_dist = 999999.9 # Left-front
        self.front_dist = 999999.9 # Front
        self.rightfront_dist = 999999.9 # Right-front
        self.right_dist = 999999.9 # Right
        self.leftback_dist = 999999.9
        self.rightback_dist = 999999.9

        self.leftfront_dist_prev = 999999.9 # Left
        self.rightfront_dist_prev = 999999.9 # Left-front
        ################### ROBOT CONTROL PARAMETERS ##################

        # Maximum forward speed of the robot in meters per second
        # Any faster than this and the robot risks falling over.
        self.forward_speed = 0.2
        self.turning_forward = 0.03



        ############# WALL FOLLOWING MODE PARAMETERS ##################


        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.0 # Fast turn
        self.turning_speed_wf_slow = 0.35 # Slow turn

        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.7 # in meters

        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.5 # in meters

        #

    def scan_callback(self, msg):

        #self.get_logger().info('L:%f' % (len(msg.ranges)))
        self.left_dist = msg.ranges[60]
        self.leftfront_dist = msg.ranges[30]
        self.leftback_dist = msg.ranges[90]
        self.front_dist = msg.ranges[0]
        self.rightfront_dist = msg.ranges[-30]
        self.right_dist = msg.ranges[-60]
        self.rightback_dist = msg.ranges[-90]

        if math.isnan(self.left_dist):
            self.left_dist = 7.6

        if math.isnan(self.leftfront_dist):
            self.leftfront_dist = 7.6

        if math.isnan(self.leftback_dist):
            self.leftback_dist = 7.6


        if math.isnan(self.front_dist):
            self.front_dist = 7.6


        if math.isnan(self.rightfront_dist):
            self.rightfront_dist = 7.6


        if math.isnan(self.right_dist):
            self.right_dist = 7.6


        if math.isnan(self.rightback_dist):
            self.rightback_dist = 7.6
        # The total number of laser rays. Used for testing.
        # number_of_laser_rays = str(len(msg.ranges))

        #Print the distance values (in meters) for testing

        #
        # self.get_logger().info('L:%f LF:%f LB:%f F:%f RF:%f R:%f RB:%f'  % (
        #   self.left_dist,
        #   self.leftfront_dist,
        #   self.leftback_dist,
        #   self.front_dist,
        #   self.rightfront_dist,
        #   self.right_dist,
        #   self.rightback_dist))

        self.follow_wall()


    def follow_wall(self):

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0


        d = self.dist_thresh_wf



        differenceLeft = self.leftback_dist - self.leftfront_dist + 0.2
        differenceRight = self.rightback_dist - self.rightfront_dist



        if (self.left_dist > d and self.leftfront_dist > d) and self.front_dist > d and (self.right_dist > d and self.rightfront_dist > d):
            self.get_logger().info('Random')

            if self.leftfront_dist_prev + 1< self.left_dist:
                msg.linear.x = self.forward_speed
                msg.angular.z = -self.turning_speed_wf_slow

            elif self.rightfront_dist_prev + 1 < self.right_dist:
                msg.linear.x = self.forward_speed
                msg.angular.z = self.turning_speed_wf_slow
            else:
                msg.linear.x = self.forward_speed


        elif  (self.left_dist < d or self.leftfront_dist < d) and self.front_dist < (d-0.2) and self.right_dist > d:
            self.get_logger().info('Corner Left')
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_fast

        elif  (self.right_dist < d or self.rightfront_dist < d) and self.front_dist < (d-0.2) and self.left_dist > d:
            self.get_logger().info('Corner Right')
            msg.angular.z = self.turning_speed_wf_fast

        elif  (self.left_dist < d) and self.front_dist > d and self.right_dist > d:
            self.get_logger().info('Left Wall Follow')
            msg.linear.x = self.forward_speed
            if self.left_dist > (d-0.09) and self.leftfront_dist > (d-0.09):
                msg.angular.z = self.turning_speed_wf_slow
                self.get_logger().info('A')
            elif self.left_dist < (self.dist_too_close_to_wall) and self.leftfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = self.turning_speed_wf_slow + 0.2
                self.get_logger().info('B')
            elif differenceLeft > 0.4:
                msg.angular.z = self.turning_speed_wf_slow+0.3
                self.get_logger().info('C')
                self.get_logger().info('%f' % (differenceLeft))
            elif differenceLeft <- 0.4:
                msg.angular.z = -self.turning_speed_wf_slow -0.3
                self.get_logger().info('D')
                self.get_logger().info('%f' % (differenceLeft))

        elif  (self.right_dist < d or self.rightfront_dist < d) and self.front_dist > d and self.left_dist > d:
            self.get_logger().info('Right Wall Follow')
            msg.linear.x = self.forward_speed
            if self.right_dist > (d-0.09) and self.rightfront_dist > (d-0.09):
                msg.angular.z = -self.turning_speed_wf_slow
                self.get_logger().info('A')
            elif self.right_dist < (self.dist_too_close_to_wall) and self.rightfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = self.turning_speed_wf_slow
                self.get_logger().info('B')
            elif differenceRight > 0.2:
                msg.angular.z = self.turning_speed_wf_slow
                self.get_logger().info('C')
            elif differenceRight <- 0.2:
                msg.angular.z = -self.turning_speed_wf_slow
                self.get_logger().info('D')



        elif self.leftfront_dist < 0.7 and self.front_dist < 0.7 and self.rightfront_dist < 0.7:
            self.get_logger().info('Corner Stuck')
            msg.angular.z = -self.turning_speed_wf_fast

        else:
            self.get_logger().info('Obstical Detected')
            #msg.angular.z = -self.turning_speed_wf_fast
            msg.linear.x = self.forward_speed


        self.leftfront_dist_prev = self.left_dist
        self.rightfront_dist_prev = self.right_dist
        self.publisher_.publish(msg)

def main(args=None):

    # Initialize rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()

    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
