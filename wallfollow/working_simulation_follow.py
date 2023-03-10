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
        # This node subscribes to messages of type Float64MultiArray
        # over a topic named: /en613/state_est
        # The message represents the current estimated state:
        #   [x, y, yaw]
        # The callback function is called as soon as a message
        # is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/state_est',
            self.state_estimate_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)


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
        self.forward_speed = 0.08

        # Current position and orientation of the robot in the global
        # reference frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # By changing the value of self.robot_mode, you can alter what
        # the robot will do when the program is launched.
        #   "obstacle avoidance mode": Robot will avoid obstacles
        #   "go to goal mode": Robot will head to an x,y coordinate
        #   "wall following mode": Robot will follow a wall
        self.robot_mode = "wall following mode"

        ############# OBSTACLE AVOIDANCE MODE PARAMETERS ##############

        # Obstacle detection distance threshold
        self.dist_thresh_obs = 0.35 # in meters

        # Maximum left-turning speed
        self.turning_speed = 0.25# rad/s



        ############# WALL FOLLOWING MODE PARAMETERS ##################
        # Finite states for the wall following mode
        #   "turn left": Robot turns towards the left
        #   "search for wall": Robot tries to locate the wall
        #   "follow wall": Robot moves parallel to the wall
        self.wall_following_state = "follow wall"

        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.0 # Fast turn
        self.turning_speed_wf_slow = 0.5 # Slow turn

        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.6 # in meters

        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.4 # in meters

        #

    def scan_callback(self, msg):
        """
        This method gets called every time a LaserScan message is
        received on the /en613/scan ROS topic
        """
        # Read the laser scan data that indicates distances
        # to obstacles (e.g. wall) in meters and extract
        # 5 distinct laser readings to work with.
        # Each reading is separated by 45 degrees.
        # Assumes 181 laser readings, separated by 1 degree.
        # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
        self.left_dist = msg.ranges[90]
        self.leftfront_dist = msg.ranges[45]
        self.leftback_dist = msg.ranges[135]
        self.front_dist = msg.ranges[0]
        self.rightfront_dist = msg.ranges[315]
        self.right_dist = msg.ranges[270]
        self.rightback_dist = msg.ranges[225]

        #The total number of laser rays. Used for testing.
        #number_of_laser_rays = str(len(msg.ranges))

        #Print the distance values (in meters) for testing
        self.get_logger().info('L:%f LF:%f F:%f RF:%f R:%f' % (
          self.left_dist,
          self.leftfront_dist,
          self.front_dist,
          self.rightfront_dist,
          self.right_dist))

    def state_estimate_callback(self, msg):

        # Update the current estimated state in the global reference frame
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]

        # Wait until we have received some goal destinations.
        self.follow_wall()


    def follow_wall(self):
        """
        This method causes the robot to follow the boundary of a wall.
        """
        # Create a geometry_msgs/Twist message
        #self.get_logger().info('follow_wall')
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0


        d = self.dist_thresh_wf

        # if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
        #     self.wall_following_state = "search for wall"
        #     msg.linear.x = self.forward_speed
        #     msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
        #
        # elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
        #     self.wall_following_state = "turn left"
        #     msg.angular.z = self.turning_speed_wf_fast
        #
        #
        # elif (self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d):
        #     if (self.rightfront_dist < self.dist_too_close_to_wall):
        #         # Getting too close to the wall
        #         self.wall_following_state = "turn left"
        #         msg.linear.x = self.forward_speed
        #         msg.angular.z = self.turning_speed_wf_fast
        #     else:
        #         # Go straight ahead
        #         self.wall_following_state = "follow wall"
        #         msg.linear.x = self.forward_speed
        #
        # # elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
        # #     self.wall_following_state = "search for wall"
        # #     msg.linear.x = self.forward_speed
        # #     msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
        #
        # elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
        #     self.wall_following_state = "turn left"
        #     msg.angular.z = self.turning_speed_wf_fast
        #
        # elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
        #     self.wall_following_state = "turn left"
        #     msg.angular.z = self.turning_speed_wf_fast
        #
        # elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
        #     self.wall_following_state = "turn left"
        #     msg.angular.z = self.turning_speed_wf_fast
        #
        # elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
        #     self.wall_following_state = "search for wall"
        #     msg.linear.x = self.forward_speed
        #     msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
        #
        # else:
        #     pass



        differenceLeft = self.left_dist - self.leftfront_dist
        differenceRight = self.right_dist - self.rightfront_dist

        #
        if (self.left_dist > d and self.leftfront_dist > d) and self.front_dist > d and (self.right_dist > d and self.rightfront_dist > d):


            if self.leftfront_dist_prev +1 < self.left_dist:
                self.get_logger().info('L:%f LF:%f' % (self.leftfront_dist_prev, 100 + self.leftfront_dist))
                msg.linear.x = self.forward_speed +0.1
                msg.angular.z = -self.turning_speed_wf_fast

            elif self.rightfront_dist_prev + 1 < self.right_dist:
                msg.linear.x = self.forward_speed + 0.1
                msg.angular.z = self.turning_speed_wf_fast
            else:
                msg.linear.x = self.forward_speed +0.1


        elif  (self.left_dist < d or self.leftfront_dist < d) and self.front_dist < d and self.right_dist > d:
            msg.angular.z = -self.turning_speed_wf_fast

        elif  (self.right_dist < d or self.rightfront_dist < d) and self.front_dist < d and self.left_dist > d:
            msg.angular.z = self.turning_speed_wf_fast

        elif  (self.left_dist < d or self.leftfront_dist < d) and self.front_dist > d and self.right_dist > d:
            self.get_logger().info('Left Wall Follow')
            msg.linear.x = self.forward_speed
            if self.left_dist > (d-0.05) and self.leftfront_dist > (d-0.05):
                msg.angular.z = self.turning_speed_wf_slow
            elif self.left_dist < (self.dist_too_close_to_wall) and self.leftfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = -self.turning_speed_wf_slow
            elif differenceLeft > 0.02:
                msg.angular.z = -self.turning_speed_wf_slow
            elif differenceLeft <- 0.02:
                msg.angular.z = self.turning_speed_wf_slow

        elif  (self.right_dist < d or self.rightfront_dist < d) and self.front_dist > d and self.left_dist > d:
            self.get_logger().info('Right Wall Follow')
            msg.linear.x = self.forward_speed
            if self.right_dist > (d-0.05) and self.rightfront_dist > (d-0.05):
                msg.angular.z = -self.turning_speed_wf_slow
            elif self.right_dist < (self.dist_too_close_to_wall) and self.rightfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = self.turning_speed_wf_slow
            elif differenceRight > 0.02:
                msg.angular.z = -self.turning_speed_wf_slow
            elif differenceRight <- 0.02:
                msg.angular.z = -self.turning_speed_wf_slow
        elif self.leftfront_dist < 0.7 and self.front_dist < 0.7 and self.rightfront_dist < 0.7:
            msg.angular.z = -self.turning_speed_wf_fast
        else:
            msg.linear.x = 0.06




        # if self.left_dist > d and self.front_dist > d and self.right_dist < d:
        #     self.get_logger().info('Right Wall Follow')
        #     msg.linear.x = self.forward_speed
        #     if self.right_dist > (d-0.1):
        #         msg.angular.z = -self.turning_speed_wf_slow
        #     elif self.right_dist <= self.dist_too_close_to_wall:
        #         msg.angular.z = self.turning_speed_wf_slow
        #
        # if self.left_dist < d and self.front_dist < d and self.right_dist > d:
        #     self.get_logger().info('Turn Right')
        #     msg.angular.z = - self.turning_speed_wf_slow
        #
        # if self.left_dist > d and self.front_dist < d and self.right_dist < d:
        #     self.get_logger().info('Turn Left')
        #     msg.angular.z = self.turning_speed_wf_slow








        self.leftfront_dist_prev = self.left_dist
        self.rightfront_dist_prev = self.right_dist
        # Send velocity command to the robot
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
