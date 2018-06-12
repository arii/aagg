#!/bin/bash

#this is like robot start, but brings up things separately.

#when developing the RT controllers for the PR2, it's often simpler to bring-up the robot again as opposed to reloading the controllers (in fact I haven't gotten this to work yet).
#however, if you robot stop robot start, you get a new roscore and any existing nodes (including rviz) get borked, and you need to restart them.
#with this, you can keep the roscore running and bring down and up the robot.
#this is just a script which automates opening a bunch of terminal windows and running commands in those windows.

interactive="shopt -q login_shell && echo 'Login shell' || echo 'Not login shell'
[[ $- == *i* ]] && echo 'Interactive' || echo 'Not interactive'"
source ~/.bashrc

#tmux new -d -s pr2mux 'echo "roscore"; $interactive; roscore; bash' \; \
tmux new -d -s pr2mux 'echo "roscore"; rviz; bash' \; \
    new-window -d -n openni 'echo "openni"; sleep 2; roslaunch openni_launch openni.launch rgb_frame_id:=base_link'\; \
    new-window -d -n sidebyside 'echo "side"; sleep 4; rosrun recording_stuff side_by_side.py '\; \
	new-window -d  -n robot_dash 'echo "dashboard"; sleep 2; rosrun rqt_pr2_dashboard rqt_pr2_dashboard; bash' \; \
		attach \;

