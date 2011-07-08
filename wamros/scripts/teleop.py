#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: TELEOP.PY
Date: Friday, June 10 2011
Description: Simple tele-operation for cartesian space control.
"""

import roslib; roslib.load_manifest('wamros')
import rospy
import curses
from wam_msgs.msg import CartesianCoordinates
import numpy as np


current_pose = np.zeros(3)
current_orient = np.zeros(3)
dirty = False

def ct_callback(data):
    global current_pose
    global current_orient
    current_pose[0] = data.position[0]
    current_pose[1] = data.position[1]
    current_pose[2] = data.position[2]
    current_orient[0] = data.euler[0]
    current_orient[1] = data.euler[1]
    current_orient[2] = data.euler[2]

def get_new_command():
    global current_pose
    global current_orient
    global window
    global dirty

    cmd = CartesianCoordinates()
    cmd.position = current_pose[:] # copy
    cmd.euler = current_orient[:]

    window.addstr("a : left, d : right, w : forward, s : back, e : up, q : down")
    window.addstr(5,0,"%s" % str(cmd))

    c = window.getch()


    if c == ord('a'):
        window.addstr(2,0,"LEFT   ")
        cmd.position[1] += 0.01
    elif c == ord('d'):
        window.addstr(2,0,"RIGHT  ")
        cmd.position[1] -= 0.01
    elif c == ord('q'):
        window.addstr(2,0,"DOWN   ")
        cmd.position[2] -= 0.01
    elif c == ord('e'):
        window.addstr(2,0,"UP     ")
        cmd.position[2] += 0.01
    elif c == ord('w'):
        window.addstr(2,0,"FORWARD")
        cmd.position[0] += 0.01
    elif c == ord('s'):
        window.addstr(2,0,"BACK   ")
        cmd.position[0] -= 0.01

    dirty = True

    return cmd

def cleanup():
    global window
    curses.nocbreak(); window.keypad(0); curses.echo()
    curses.endwin()

def teleop():
    global window
    global dirty

    # our rospy topics
    sub = rospy.Subscriber('/wam/cartesian_coordinates', CartesianCoordinates, ct_callback)
    pub = rospy.Publisher('/wam/cartesian_command', CartesianCoordinates)
    rospy.init_node('teleop')
    rospy.on_shutdown(cleanup)

    # # init simple curses interface
    window = curses.initscr()
    window.keypad(0)
    curses.noecho()
    curses.cbreak()

    # main loop
    while not rospy.is_shutdown():
        cmd = get_new_command()
        window.refresh()

        if (dirty):
            pub.publish(cmd)
            dirty = False



if __name__ == '__main__':
    teleop()
