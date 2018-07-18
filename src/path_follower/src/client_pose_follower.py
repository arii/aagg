#!/usr/bin/env python

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header

import rospy
rospy.init_node("simple_pose_publisher")
arm = 'r'
root_frame = 'torso_lift_link'
tool_frame = '%s_gripper_tool_frame' % arm
pos = (.7, -0.18, 0.29)
quat = (0, 0, 0, 1)

pub = rospy.Publisher("pose_generated", \
#pub = rospy.Publisher("%s_cart/command_pose" % arm,\
                        PoseStamped, queue_size=1)


def stamp_pose((pos,quat), root_frame):
    ps = PoseStamped( 
            Header(0,rospy.Time(0),root_frame),\
            Pose(Point(*pos),\
            Quaternion(*quat)))
    return ps

rospy.sleep(1)
cmd = stamp_pose( (pos,quat), root_frame )
pub.publish(cmd)

