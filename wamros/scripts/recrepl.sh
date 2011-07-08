#!/bin/bash -x

# This script sets the robot to passive, starts recording, then replays the recording using rosbag.
rosservice call /wam/switchMode 0

rosservice call /wam/active_passive '[0,0,0,0,0,0,0]'

echo "Press any key to start recording. Press Ctrl-C to stop." 
read stuff

rostopic echo -n 1 /wam/joints_sensed > tmp.echo
rosbag record /wam/joints_sensed -O tmp.bag

echo "End of recording. Press any key to move to first position."
read stuff


# move to first position
pos=`cat tmp.echo | perl -ne 'if (m/^radians: (.*)/) {print $1;}'`
rosservice call /wam/moveToPos "{pos: {radians: $pos}}"

# Note:
# wam/status is 1 during moveToPos
# wam/status is 2 while following joints_sensed command
# wam/status is 0 while idle

echo "When robot is finished press any key to replay action."
read stuff

rosbag play tmp.bag /wam/joints_sensed:=wam/joints_command

echo "Finished!"
