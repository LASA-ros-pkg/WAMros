#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: TELEOP.PY
Date: Friday, June 10 2011
Description: Simple tele-operation for cartesian space control.
"""


#!/usr/bin/env python
import roslib; roslib.load_manifest('wamros')
import rospy
import curses
from wam_msgs.msg import CartesianTargets

current_target = [300,200]
window = None

def ct_callback(data):
    global current_target
    current_target[0] = data.pos[0]
    current_target[1] = data.pos[1]

def get_new_command():
    global current_target
    global window

    cmd = current_target[:] # copy
    c = window.getch() # waits for key input


    if c == curses.KEY_UP:
        window.addstr(2,0,"KEY_UP    ")
        cmd[1] += 10
    elif c == curses.KEY_DOWN:
        window.addstr(2,0,"KEY_DOWN  ")
        cmd[1] -= 10
    elif c == curses.KEY_LEFT:
        window.addstr(2,0,"KEY_LEFT  ")
        cmd[0] += 10
    elif c == curses.KEY_RIGHT:
        window.addstr(2,0,"KEY_RIGHT ")
        cmd[0] -= 10


    return cmd

def cleanup():
    global window
    curses.nocbreak(); window.keypad(0); curses.echo()
    curses.endwin()

def teleop():
    global window

    # our rospy topics
    sub = rospy.Subscriber('/wam/cartesian_targets', CartesianTargets, ct_callback)
    pub = rospy.Publisher('/wam/cartesian_command', CartesianTargets)
    rospy.init_node('teleop')
    rospy.on_shutdown(cleanup)

    # init simple curses interface
    window = curses.initscr()
    window.addstr("Use the arrow keys to move the robot tool and (Ctrl-C) to exit.")
    window.keypad(1)
    curses.noecho()
    curses.cbreak()

    # main loop
    while not rospy.is_shutdown():

        cmd = get_new_command()
        window.refresh()
        pub.publish(CartesianTargets(cmd))
        rospy.sleep(0.1)

    
if __name__ == '__main__':
    teleop()
