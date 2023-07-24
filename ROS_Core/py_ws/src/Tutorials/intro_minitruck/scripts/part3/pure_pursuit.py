#!/usr/bin/env python

import threading
import rospy
import numpy as np
from utils import RealtimeBuffer, get_ros_param, State2D, GeneratePwm

from Modules.ROS_msgs.msg import ServoMsg 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Pure_Pursuit():
    '''
    Main class for the controller
    '''
    def __init__(self):
        '''
        Constructor for the PurePursuitController class
        '''
        # Initialize the real-time buffer for the state and goal
        self.state_buffer = RealtimeBuffer()
        self.goal_buffer = RealtimeBuffer()
        
        # Initialize the PWM converter
        self.pwm_converter = GeneratePwm()

        # Read Parameters from the parameter server
        self.read_parameters()

        # Setup the publisher and subscriber
        self.setup_publisher()
        self.setup_subscriber()

        # start planning thread
        # We use a thread to run the planning loop so that we can continuously read the odometry
        # and goal message in the callback function without being blocked by the planning loop
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
        self.control_pub = rospy.Publisher(self.control_topic, ServoMsg, queue_size=1)
        ########################### END OF TODO 1#################################
        
            
    def setup_subscriber(self):
        '''
        This function sets up the subscriber for the odometry and goal message
        '''
        # This set up a subscriber for the goal you click on the rviz
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        
        ################## TODO: 2. Set up a subscriber for the odometry message###################
        # Create a subscriber:
        #   - subscribes to the topic <self.odom_topic>
        #   - has message type <Odometry> (nav_msgs.msg.Odometry) 
        #   - with callback function <self.odometry_callback>, which has already been implemented
        #   - with queue size 1
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=1)

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

    def goal_callback(self, goal_msg: PoseStamped):
        """
        Subscriber callback function of the robot goal point
        Parameters:
            goal_msg: goal message with type geometry_msgs.msg.PoseStamped
        """
        ############## TODO: 3. Fill in the subscriber callback function ###################
        # 1. Inspect the data structure of the goal message <geometry_msgs.msg.PoseStamped>
        #   Hint: check the message data structure using the command
        #       rosmsg show geometry_msgs/PoseStamped
        #    Check the properties of a goal_msg!
        # 2. Retrieve the goal from the goal message 
        #   and create a 3-dim numpy array [x,y,1] using np
        # 3. add the goal to the buffer (self.goal_buffer)
        
        goal_x = goal_msg.pose.position.x # TO BE FILLED
        goal_y = goal_msg.pose.position.y # TO BE FILLED
        goal = np.array([goal_x, goal_y, 1])
        self.goal_buffer.writeFromNonRT(goal)
        
        ########################### END OF TODO 3 #################################
        
        # Log the goal to the console using "rospy.loginfo"
        rospy.loginfo(f"Received a new goal [{np.round(goal_x, 3)}, {np.round(goal_y,3)}]")
        
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
        
        servo_msg = ServoMsg(throttle = throttle, steer = steer)
        servo_msg.header.stamp = rospy.Time.now()
        self.control_pub.publish(servo_msg)
        
    def planning_thread(self):
        '''
        Main thread for the planning
        '''
        rospy.loginfo("Planning thread started waiting for ROS service calls...")
        while not rospy.is_shutdown():
            
            if self.state_buffer.new_data_available:

                # read the current state and goal from the buffer
                state_cur = self.state_buffer.readFromRT()
                goal_cur = self.goal_buffer.readFromRT()
            
                # current longitudinal velocity
                vel_cur = state_cur.v_long 

                # check if a goal is available
                if goal_cur is not None:
                    
                    # First, transform the goal to the robot frame
                    goal_robot = np.linalg.inv(state_cur.transformation_matrix()).dot(goal_cur)
                    
                    # relative heading angle of the goal wrt the car
                    alpha = np.arctan2(goal_robot[1], goal_robot[0])
                    
                    # relative distance between the car and the goal
                    dis2goal = np.sqrt(goal_robot[0]**2 + goal_robot[1]**2)

                    ########################## Pure Pursuit Algorithm ###################
                    
                    # 1. Check if the goal is close enough, then stop the car by applying a negative
                    #  acceleration (eg: -1 m/s^2) and zero steering angle. 
                    #  Then, continue to the next iteration

                    if dis2goal <= self.stop_distance:
                        self.publish_control(-1, 0, state_cur)
                        continue
                        

                    # If the target is behind the car, apply maximum steering angle 
                    # and set the reference_velocity to vel_max to do a full curve
                    if np.abs(alpha) > np.pi/2:
                        steer = np.sign(alpha) * self.max_steer
                        vel_ref = self.max_vel
                    
                    #   Otherwise
                    # - apply a pure pursuit controller for steering by assuming 
                    #   the reference path is a straight line between the car and the goal
                    # - set the reference_velocity to the minimum of vel_max and (dis2goal-self.stop_distance)
                    # - A detailed explanation of the algorithm can be found here: 
                    #   https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html
                    else:
                        ld = min(dis2goal, self.ld_max)
                        steer = np.arctan2(2*self.wheel_base*np.sin(alpha), ld)                            
                        vel_ref = min(self.max_vel, dis2goal-self.stop_distance)
                    
                    # clip the steering angle
                    steer = min(max(steer, -1*self.max_steer), self.max_steer)
                    # Apply the simple proportional controller
                    accel = self.throttle_gain * (vel_ref - vel_cur)
                    
                    # publish the control
                    self.publish_control(accel, steer, state_cur)