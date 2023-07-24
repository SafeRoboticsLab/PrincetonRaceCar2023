#!/usr/bin/env python
import rospy
from part3 import Pure_Pursuit


def main():
    rospy.init_node('pursuit_node')
    rospy.loginfo("Start Pure Pursuit node")
    
    # Initialize the PurePursuitController class
    Pure_Pursuit()
    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()
