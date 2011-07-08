#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: RUNBAG.PY
Date: Monday, July 7th, 2011
Description: Replay coordinates as control commands.
"""

import roslib
roslib.load_manifest('wamros')
import rospy
import rosbag
import sys
from wam_msgs.msg import CartesianTargets

def main():

    rospy.init_node("runbag")
    pub = rospy.Publisher('/wam/cartesian_command', CartesianTargets)

    rate = rospy.Rate(100)
    bag = rosbag.Bag(sys.argv[1])
    for topic, msg, t in bag.read_messages():
        if topic == "/wam/cartesian_coordinates":
            rospy.logdebug("Publishing %s" % str(msg.position))
            pub.publish(CartesianTargets(msg.position))
            rate.sleep()

if __name__ == '__main__':
    main()
