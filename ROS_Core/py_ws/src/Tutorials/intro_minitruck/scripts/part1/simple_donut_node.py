#!/usr/bin/env python
import rospy
from part1 import Simple_Donut


def main():
    rospy.init_node('simple_donut_node')
    rospy.loginfo("Start Simple Donut Node")
    
    # Initialize the Simple Donut class
    Simple_Donut()
    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()
