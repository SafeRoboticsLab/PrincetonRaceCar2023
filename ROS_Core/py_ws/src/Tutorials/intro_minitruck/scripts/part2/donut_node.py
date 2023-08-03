#!/usr/bin/env python

import rospy
from part2 import Donut


def main():
    rospy.init_node('donut_node')
    rospy.loginfo("Start Donut Node")
    
    # Initialize the Donut class
    Donut()
    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()
