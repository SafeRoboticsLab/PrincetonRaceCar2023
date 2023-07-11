#!/usr/bin/env python
import rospy
from controller import Donut


def main():
    rospy.init_node('donut_node')
    rospy.loginfo("Start Donut Node")
    
    # Initialize the PurePursuitController class
    Donut()
    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()
