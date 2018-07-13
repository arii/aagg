#!/usr/bin/env python

from geometry_msgs.msg import Pose, PoseStamped, Point, \
                              Quaternion, PointStamped
from nav_msgs.msg import Path

from std_msgs.msg import Header
import numpy as np
import tf
import rospy
pos = (.7, -0.18, 0.23)
quat = Quaternion(0, -.7071, 0, .7071)

class PathGenerator:
    def __init__(self, arm, root_frame, tool_frame, tf_listener=None):
        self.root_frame = root_frame
        self.tool_frame = tool_frame
        self.arm = arm
        self.pub = rospy.Publisher("%s_cart/command_pose" % arm,\
                        PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher("path_generated", Path, queue_size=1)
        self.sub = rospy.Subscriber("/clicked_point", PointStamped, \
                        self.updateOrigin)
        self.origin_updated = False
        self.origin = Point(0.6, -0.18, 0.23) #XXX todo : make this current pose
        self.quat =quat # Quaternion(0,0,0,1)

        if tf_listener is None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

    def updateOrigin(self, msg):
        if msg.header.frame_id != self.root_frame:
            rospy.warn("need to convert to root frame")
            raise NotImplementedError
        else:
            self.origin = msg.point
            rospy.loginfo("updated origin to (%.2f %.2f %.2f)" % \
                    (self.origin.x, self.origin.y, self.origin.z)
                    )
        self.circle(.1)

    def stamp_pose(self, (pos,quat)):
        ps = PoseStamped( 
            Header(0,rospy.Time(0),self.root_frame),\
            Pose(Point(*pos),\
            Quaternion(*quat)))
        return ps

    def get_header(self):
        return Header(0, rospy.Time(0), self.root_frame)

    def circle(self, r, pathnum=0):
        """ generator points around a circle with radius r"""
        rospy.loginfo("generating a circle path")
        
        origin = self.origin
        quat = self.quat
        header = self.get_header()
        
        disc =8
        path = Path()
        path.header = header
        for i in range(disc -1):
            #th =  .5*np.pi*(pathnum %4) + .5*np.pi*float(i)/disc
            th =  2*np.pi*float(i)/disc
            x = r*np.cos(th) + origin.x # +  r*(float(i)/disc)

            y = r*np.sin(th) + origin.y #+ r*(float(i)/disc)
            z = origin.z #+  ((-1)**i) * r*(float(i)/disc)
            pos = Point(x,y,z)
            ps = PoseStamped(header, Pose(pos, quat))
            path.poses.append(ps)
        
        self.path_pub.publish(path)
        self.pub.publish(path.poses[0])

if __name__=="__main__":
    rospy.init_node("path_generator")
    arm = 'r'
    root_frame = 'torso_lift_link'
    tool_frame = '%s_gripper_tool_frame' % arm

    pg = PathGenerator(arm, root_frame, tool_frame)
    rospy.sleep(.5)
    #pg.circle(.12)
    i = 0 
    while True:
        i += 1 
        pg.circle(.15, i)
        rospy.sleep(10)
    rospy.spin()

"""
rospy.sleep(1)
cmd = stamp_pose( (pos,quat), root_frame )
pub.publish(cmd)
print cmd
rospy.spin()
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

