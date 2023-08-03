#!/usr/bin/env python

import rospy
import threading
import math
from utils import RealtimeBuffer, get_ros_param, State2D, GeneratePwm

from racecar_msgs.msg import ServoMsg 
from nav_msgs.msg import Odometry
class LetterDrawingBot:
   def __init__(self):
      
      self.rate = rospy.Rate(10)  # 10 Hz
      self.state_buffer = RealtimeBuffer()

      # Initialize the PWM converter
      self.pwm_converter = GeneratePwm()

      # Read Parameters from the parameter server
      self.read_parameters()

      # Setup the control publisher
      self.setup_publisher()
      self.setup_subscriber()

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
      # Setup the publisher to control the robot using ServoMsgs
      self.control_pub = rospy.Publisher(self.control_topic, ServoMsg, queue_size=1)

   def setup_subscriber(self):
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

   def stop_robot(self, state):
      # TODO 1: Stop the robot's motion by publishing a zero command
      # Between various movements, we want the robot to come to a complete stop
      # Use the current velocity and a reference velocity to accomplish this goal

      vel_cur = state.v_long
      vel_ref = 0
      accel = self.throttle_gain * (vel_ref - vel_cur) 
      self.publish_control(accel, 0, state)

   def draw_curve(self, time_to_draw, state, clockwise):
      steer = self.max_steer if not clockwise else -self.max_steer
      start_time = rospy.Time.now()

      # TODO 2: Write a while loop that publishes control until the half or full circle is drawn.
      while rospy.Time.now() - start_time < rospy.Duration(time_to_draw):
         self.publish_control(accel = 0.0, steer = steer, state = state)
         self.rate.sleep()


   def draw_circle(self, full, state, clockwise):      
      vel_cur = state.v_long
      radius = self.wheel_base/math.tan(self.max_steer)
      # TODO 3: Based on the radius and our current velocity, how long should we curve to draw either
      # a full circle or a half circle.
      if full is True:   
         time_to_draw = 2 * (math.pi * radius) / vel_cur
      else:
         time_to_draw = 1 * (math.pi * radius) / vel_cur

      # Uses the curve command to draw a half or full circle
      self.draw_curve(time_to_draw, state, clockwise)


   def draw_straight_line(self, time_to_draw, state):
      start_time = rospy.Time.now()
      # TODO 4: Use your code from todo 2 in order to draw a straight line
      while rospy.Time.now() - start_time < rospy.Duration(time_to_draw):
         self.publish_control(0, 0.0, state)
         self.rate.sleep()

   def get_started(self, state):
      start_time = rospy.Time.now()
      # TODO 5: Use your code from todo 2 to complete the while loop below.
      while rospy.Time.now() - start_time < rospy.Duration(2):
         if (state.v_long != 0):
            self.publish_control(accel = 0, steer = 0, state = self.state_buffer.readFromRT())
         else:
            self.publish_control(accel = 0.75, steer = 0, state = None)


   def planning_thread(self):
      i = 0
      while not rospy.is_shutdown():
         if self.state_buffer.new_data_available:
            self.get_started(state = self.state_buffer.readFromRT())

            # Using a combination of your different shapes, draw a letter of your choice. 
            # Our initial provided example is the letter 's'

            self.draw_straight_line(time_to_draw=1, state = self.state_buffer.readFromRT()) 
            self.draw_circle(full = False, state = self.state_buffer.readFromRT(), clockwise = False) 
            self.draw_straight_line(time_to_draw=1, state = self.state_buffer.readFromRT()) 
            self.draw_circle(full = False, state = self.state_buffer.readFromRT(), clockwise = True)
            self.draw_straight_line(time_to_draw=1, state = self.state_buffer.readFromRT()) 
            self.stop_robot(state = self.state_buffer.readFromRT())