#!/usr/bin/env python

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header

import rospy
rospy.init_node("simple_pose_publisher")
arm = 'r'
root_frame = 'base_link'
tool_frame = '%s_gripper_tool_frame' % arm
pos = (.97, -0.18, 0.99)
quat = (0, 0, 0, 1)

pub = rospy.Publisher("%s_cart/command_pose" % arm,\
                        PoseStamped, queue_size=1)


def stamp_pose((pos,quat), root_frame):
    ps = PoseStamped( 
            Header(0,rospy.Time(0),root_frame),\
            Pose(Point(*pos),\
            Quaternion(*quat)))
    return ps

cmd = stamp_pose( (pos,quat), root_frame )
pub.publish(cmd)
print cmd
rospy.spin()
"""
def command_delta(x,y,z):
    pos, quat = get_pose()
    pos[0] += x
    pos[1] += y
    pos[2] += z
    cmd = stamp_pose( (pos,quat))
    pub.publish(cmd)

command_delta( 0,0,.05)
command_delta( 0,0,-.05)
"""

