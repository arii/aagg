#!/usr/bin/env python

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from actionlib import SimpleActionClient as SAC
import path_follower.msg
import rospy
rospy.init_node("simple_pose_publisher")
arm = 'r'
root_frame = 'torso_lift_link'
tool_frame = '%s_gripper_tool_frame' % arm
pos = (.5, -0.18, 0.29)
quat = (0, 0, 0, 1)

#pub = rospy.Publisher("pose_generated", \
#pub = rospy.Publisher("%s_cart/command_pose" % arm,\
#                        PoseStamped, queue_size=1)


def stamp_pose((pos,quat), root_frame):
    ps = PoseStamped( 
            Header(0,rospy.Time(0),root_frame),\
            Pose(Point(*pos),\
            Quaternion(*quat)))
    return ps

#rospy.sleep(1)
cmd = stamp_pose( (pos,quat), root_frame )
#pub.publish(cmd)


client = SAC("pose_follower", path_follower.msg.GagaPoseAction)

client.wait_for_server()
goal = path_follower.msg.GagaPoseGoal()
goal.pose = cmd
goal.timeout = rospy.Duration(0)
goal.lookahead = .01
goal.goal_cartesian_tolerance = 0.006

client.send_goal(goal)
client.wait_for_result()
print client.get_result()

for i in range(10):
    if i %2 == 0:
        pos = (.5, -0.18, 0.29)
        quat = (0, 0, 0, 1)
    else:
        pos = (.7, -0.18, 0.29)
        quat = (1, 0, 0, 0)
    cmd = stamp_pose( (pos,quat), root_frame )
    goal.pose = cmd
    client.send_goal(goal)
    client.wait_for_result()
    res = client.get_state()
    print res


