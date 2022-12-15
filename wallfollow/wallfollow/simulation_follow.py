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

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.timer_period = 0.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.follow_wall)



        #################33 INITIALIZING THE SENSOR VALUES ################
        self.left_dist = 999999.9 # Left
        self.leftfront_dist = 999999.9 # Left-front
        self.front_dist = 999999.9 # Front
        self.rightfront_dist = 999999.9 # Right-front
        self.right_dist = 999999.9 # Right
        self.leftback_dist = 999999.9 #Left Back
        self.rightback_dist = 999999.9 #Right Back

        self.leftfront_dist_prev = 999999.9 # Left Previous Distance
        self.rightfront_dist_prev = 999999.9 # Left-front Previous Distance
        ####################################################################

        ################### ROBOT CONTROL PARAMETERS #######################

        # Maximum forward speed of the robot in meters per second
        self.forward_speed = 0.08

        ################################################################
        ############# WALL FOLLOWING MODE PARAMETERS ##################


        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.0 # Fast turn
        self.turning_speed_wf_slow = 0.5 # Slow turn

        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.6 # in meters

        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.4 # in meters

        ##################################################################

    def scan_callback(self, msg):

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
        # self.get_logger().info('L:%f LF:%f F:%f RF:%f R:%f' % (
        #   self.left_dist,
        #   self.leftfront_dist,
        #   self.front_dist,
        #   self.rightfront_dist,
        #   self.right_dist))


    def follow_wall(self):


        # Creating a twist message and INITIALIZING each value to 0
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        #Initialize our distance from the wall to the distance threshold that we want
        d = self.dist_thresh_wf

        #################################### OUR ALGORITHM ###############################################################################




        # Checking differences to see if the robot is parallel to the wall or not
        differenceLeft = self.left_dist - self.leftfront_dist
        differenceRight = self.right_dist - self.rightfront_dist

        # If all the values from sensors are bigger than threshold then just move in a random direction until a wall is detected
        if (self.left_dist > d and self.leftfront_dist > d) and self.front_dist > d and (self.right_dist > d and self.rightfront_dist > d):
            self.get_logger().info('Finding a Wall')


            # Condition for left door
            if self.leftfront_dist_prev +1 < self.left_dist:
                msg.linear.x = self.forward_speed +0.1
                msg.angular.z = -self.turning_speed_wf_fast

            # Condition for right door
            elif self.rightfront_dist_prev + 1 < self.right_dist:
                msg.linear.x = self.forward_speed + 0.1
                msg.angular.z = self.turning_speed_wf_fast
            else:
                msg.linear.x = self.forward_speed +0.1


        #Condition to detect the left corner
        elif  (self.left_dist < d or self.leftfront_dist < d) and self.front_dist < d and self.right_dist > d:
            self.get_logger().info('Left Corner')
            msg.angular.z = -self.turning_speed_wf_fast

        # Condition to detect the right corner
        elif  (self.right_dist < d or self.rightfront_dist < d) and self.front_dist < d and self.left_dist > d:
            self.get_logger().info('Right Corner')
            msg.angular.z = self.turning_speed_wf_fast

        #Condition for detecting a wall to the left of the robot
        elif  (self.left_dist < d or self.leftfront_dist < d) and self.front_dist > d and self.right_dist > d:
            self.get_logger().info('Left Wall Follow')
            msg.linear.x = self.forward_speed

            #If it is going away from the wall then turn left
            if self.left_dist > (d-0.05) and self.leftfront_dist > (d-0.05):
                msg.angular.z = self.turning_speed_wf_slow

            #If it is going too close to the wall then turn right
            elif self.left_dist < (self.dist_too_close_to_wall) and self.leftfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = -self.turning_speed_wf_slow

            #if the robot is not parallel and is moving closer to the wall, then turn right
            elif differenceLeft > 0.02:
                msg.angular.z = -self.turning_speed_wf_slow

            #if the robot is not parallel to the wall and is moving away then turn left
            elif differenceLeft <- 0.02:
                msg.angular.z = self.turning_speed_wf_slow

        #Condition for detecting a wall to the right of the robot
        elif  (self.right_dist < d or self.rightfront_dist < d) and self.front_dist > d and self.left_dist > d:
            self.get_logger().info('Right Wall Follow')
            msg.linear.x = self.forward_speed

            #if it is going away from the wall then turn right
            if self.right_dist > (d-0.05) and self.rightfront_dist > (d-0.05):
                msg.angular.z = -self.turning_speed_wf_slow

            #if it is going towards the wall then turn left
            elif self.right_dist < (self.dist_too_close_to_wall) and self.rightfront_dist < (self.dist_too_close_to_wall):
                msg.angular.z = self.turning_speed_wf_slow

            #if the robot is not parallel and is moving closer to the wall then turn left
            elif differenceRight > 0.02:
                msg.angular.z = -self.turning_speed_wf_slow

            #if the obot is not parallel and is moving away from the wall then turn right
            elif differenceRight <- 0.02:
                msg.angular.z = -self.turning_speed_wf_slow

        #Condition for if the robot is stuck in a corner
        elif self.leftfront_dist < 0.7 and self.front_dist < 0.7 and self.rightfront_dist < 0.7:
            self.get_logger().info('Corner Stuck')
            msg.angular.z = -self.turning_speed_wf_fast

            #if none of the above conditions trigger, then move straight
        else:
            self.get_logger().info('Move Slower')
            msg.linear.x = 0.06


        #Save new previous values
        self.leftfront_dist_prev = self.left_dist
        self.rightfront_dist_prev = self.right_dist

        ###################################################################################################################################
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
