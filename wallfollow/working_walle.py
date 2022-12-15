import math
import rclpy
from time import sleep
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class Controller(Node):


    def __init__(self):
        super().__init__('Controller')

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

        self.timer_period = 0.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.follow_wall)


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
        self.forward_speed = 0.06


        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.0 # Fast turn
        self.turning_speed_wf_slow = 0.35 # Slow turn

        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.8 # in meters

        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.55 # in meters

        #

    def scan_callback(self, msg):

        # Read the laser scan data that indicates distances
        # to obstacles (e.g. wall) in meters and extract
        # 5 distinct laser readings to work with.
        # Each reading is separated by 45 degrees.
        # Assumes 181 laser readings, separated by 1 degree.
        # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
        self.left_dist = msg.ranges[50]
        self.leftfront_dist = msg.ranges[25]
        self.leftback_dist = msg.ranges[75]
        self.front_dist = msg.ranges[0]
        self.rightfront_dist = msg.ranges[-25]
        self.right_dist = msg.ranges[-50]
        self.rightback_dist = msg.ranges[-75]

        #The total number of laser rays. Used for testing.
        #number_of_laser_rays = str(len(msg.ranges))

        #Print the distance values (in meters) for testing
        self.get_logger().info('L:%f LF:%f F:%f RF:%f R:%f' % (
          self.left_dist,
          self.leftfront_dist,
          self.front_dist,
          self.rightfront_dist,
          self.right_dist))


        # self.follow_wall()


    def follow_wall(self):

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0


        d = self.dist_thresh_wf


################# OUR IMPLEMENTATION ####################################


        differenceLeft = self.left_dist - self.leftfront_dist
        differenceRight = self.right_dist - self.rightfront_dist

        #
        if (self.left_dist > d and self.leftfront_dist > d) and self.front_dist > d and (self.right_dist > d and self.rightfront_dist > d):
            self.get_logger().info('Random')

            if self.leftfront_dist_prev + 1< self.left_dist:
                # self.get_logger().info('L:%f LF:%f' % (self.leftfront_dist_prev, 100 + self.leftfront_dist))
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
            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_fast

        elif  (self.left_dist < d) and self.front_dist > d and self.right_dist > d:
            self.get_logger().info('Left Wall Follow')
            msg.linear.x = self.forward_speed
            if self.left_dist > (d-0.09) and self.leftfront_dist > (d-0.09):
                msg.angular.z = self.turning_speed_wf_slow
            elif self.left_dist < (self.dist_too_close_to_wall) and self.leftfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = -self.turning_speed_wf_slow
            elif differenceLeft > 0.03:
                msg.angular.z = -self.turning_speed_wf_slow + 0.23
            elif differenceLeft <- 0.03:
                msg.angular.z = self.turning_speed_wf_slow -0.23

        elif  (self.right_dist < d or self.rightfront_dist < d) and self.front_dist > d and self.left_dist > d:
            self.get_logger().info('Right Wall Follow')
            msg.linear.x = self.forward_speed
            if self.right_dist > (d-0.09) and self.rightfront_dist > (d-0.09):
                msg.angular.z = -self.turning_speed_wf_slow
            elif self.right_dist < (self.dist_too_close_to_wall) and self.rightfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = self.turning_speed_wf_slow
            elif differenceRight > 0.03:
                msg.angular.z = self.turning_speed_wf_slow - 0.23
            elif differenceRight <- 0.03:
                msg.angular.z = -self.turning_speed_wf_slow +0.23

        elif self.leftfront_dist < 0.7 and self.front_dist < 0.7 and self.rightfront_dist < 0.7:
            self.get_logger().info('Corner Stuck')
            msg.angular.z = -self.turning_speed_wf_fast

        else:
            self.get_logger().info('Move Straight')
            # msg.angular.z = -self.turning_speed_wf_fast
            msg.linear.x = self.forward_speed


        self.leftfront_dist_prev = self.left_dist
        self.rightfront_dist_prev = self.right_dist
        # msg.linear.x = 0.2


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
