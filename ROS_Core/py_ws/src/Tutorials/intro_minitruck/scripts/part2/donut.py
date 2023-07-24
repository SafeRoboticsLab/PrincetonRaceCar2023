#!/usr/bin/env python

import threading
import rospy
from utils import get_ros_param, RealtimeBuffer, GeneratePwm, State2D

from Modules.ROS_msgs.msg import ServoMsg 
from nav_msgs.msg import Odometry

class Donut():

    '''
    Main class for the controller
    '''
    def __init__(self):
        '''
        Constructor for the PurePursuitController class
        '''
        # Initialize the real-time buffer for the state
        self.state_buffer = RealtimeBuffer()

        # Initialize the PWM converter
        self.pwm_converter = GeneratePwm()

        # Read Parameters from the parameter server
        self.read_parameters()

        # Setup the publisher and subscriber
        self.setup_publisher()
        self.setup_subscriber()

        # start planning thread
        # We use a thread to run the planning loop so that we can continuously read the odometry
        #  in the callback function
        threading.Thread(target=self.planning_thread).start()

    def read_parameters(self):
        '''
        This function reads the parameters from the parameter server
        '''
        # ROS topic name to subscribe odometry 
        self.odom_topic = get_ros_param('~odom_topic', '/SLAM/Pose')
        
        # Read ROS topic name to publish control command
        self.control_topic = get_ros_param('~control_topic', '/Control')
        
        # Proportional gain for the throttle
        self.throttle_gain = get_ros_param('~throttle_gain', 0.05)
        
        # maximum look ahead distance
        self.ld_max = get_ros_param('~ld_max', 2)
        
        # Read controller thresholds
        self.max_steer = get_ros_param('~max_steer', 0.35)
        self.max_vel = get_ros_param('~max_vel', 0.5)
        
        # Distance threshold to stop
        self.stop_distance = get_ros_param('~stop_distance', 0.5)
        
        # Read the wheel base of the robot
        self.wheel_base = get_ros_param('~wheel_base', 0.257)
        
        # Read the simulation flag, 
        # if the flag is true, we are in simulation 
        # and no need to convert the throttle and steering angle to PWM
        self.simulation = get_ros_param('~simulation', True)
        
    def setup_publisher(self):
        '''
        This function sets up the publisher for the control command
        '''
        ################## TODO: 1. Set up a publisher for the ServoMsg message###################
        # Create a publiser - self.control_pub:
        #   - subscribes to the topic <self.control_topic>
        #   - has message type <ServoMsg> (racecar_msgs.msg.Odometry) 
        #   - with queue size 1

        self.control_pub # TO BE COMPLETED
        ########################### END OF TODO 1#################################
        
            
    def setup_subscriber(self):
        '''
        This function sets up the subscriber for the odometry message
        '''
        ################## TODO: 2. Set up a subscriber for the odometry message###################
        # Create a subscriber:
        #   - subscribes to the topic <self.odom_topic>
        #   - has message type <Odometry> (nav_msgs.msg.Odometry) 
        #   - with callback function <self.odometry_callback>, which has already been implemented
        #   - with queue size 1

        self.pose_sub # TO BE COMPLETED
        ########################### END OF TODO 2#################################
        
    def odometry_callback(self, odom_msg: Odometry):
        """
        Subscriber callback function for the robot odometry message
        Parameters:
            odom_msg: Odometry message with type nav_msgs.msg.Odometry
        """
        # Retrieve state needed for planning from the odometry message
        # [x, y, v, w, delta]
        state_cur = State2D(odom_msg = odom_msg)

        # Add the current state to the buffer
        # Planning thread will read from the buffer
        self.state_buffer.writeFromNonRT(state_cur)

        
    def publish_control(self, accel, steer, state):
        '''
        Helper function to publish the control command
        Parameters:
            accel: float, linear acceleration of the robot [m/s^2]
            steer: float, steering angle of the robot [rad]
            state: State2D, current state of the robot
        '''
        # If we are in simulation,
        # the throttle and steering angle are acceleration and steering angle
        if self.simulation:
            throttle = accel
        else:
            # If we are using robot,
            # the throttle and steering angle needs to convert to PWM signal
            throttle, steer = self.pwm_converter.convert(accel, steer, state)
            
        message = ServoMsg(throttle = throttle, steer = steer)
        message.header.stamp = rospy.Time.now()
        self.control_pub.publish(message)
        

    def planning_thread(self):
        '''
        Main thread for the planning
        '''
        rospy.loginfo("Planning thread started waiting for ROS service calls...")
        while not rospy.is_shutdown():
               
            # read the current state from the buffer
            state_cur = self.state_buffer.readFromRT()
            
            # current longitudinal velocity
            vel_cur = state_cur.v_long 

            steer = self.max_steer
            vel_ref = self.max_vel
            accel = self.throttle_gain * (vel_ref - vel_cur)
            
            self.publish_control(accel, steer, state_cur)



