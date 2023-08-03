#!/usr/bin/env python

import threading
import rospy
from utils import get_ros_param

from racecar_msgs.msg import ServoMsg 

class Simple_Donut():

    def __init__(self):
        '''
        '''
        # Read Parameters from the parameter server
        self.read_parameters()

        # Setup the publisher
        self.setup_publisher()

        # start planning thread for constant sensor reading 
        threading.Thread(target=self.planning_thread).start()


    def read_parameters(self):
        '''
        This function reads the parameters from the parameter server
        '''
        
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
        
        
    def setup_publisher(self):
        '''
        This function sets up the publisher for the control command
        '''
        self.control_pub = rospy.Publisher(self.control_topic, ServoMsg, queue_size=1)
        

    def publish_control(self, accel, steer):
        '''
        Helper function to publish the control command
        Parameters:
            accel: float, linear acceleration of the robot [m/s^2]
            steer: float, steering angle of the robot [rad]
        '''
        message = ServoMsg(throttle = accel, steer = steer)
        self.control_pub.publish(message)
    

    def planning_thread(self):
        '''
        Main thread for the planning
        '''

        while not rospy.is_shutdown():

            steer = self.max_steer
            accel = self.throttle_gain * 0.5
            self.publish_control(accel, steer)


