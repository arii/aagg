#!/usr/bin/env python

from geometry_msgs.msg import Pose, PoseStamped, Point, \
                              Quaternion, PointStamped
from nav_msgs.msg import Path

from std_msgs.msg import Header
import numpy as np
import tf
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA
from robot_mechanism_controllers.msg import JTCartesianControllerState \
        as ControllerState

"""
TODO:
    add simple action client wrapper  ?

    include quaternion interpolation

    include quaternion in distance computation

    what if curvature is too tight?/ wrap around 
    -- make sure we keep trying to move forward


"""

class PoseFollower:
    
    def __init__(self):

        self.pose_topic = rospy.get_param("~pose_topic", "pose_generated")
        self.lookahead        = float(rospy.get_param("~lookahead", ".01"))
        self.max_reacquire    = rospy.get_param("~max_reacquire", ".1")
        self.root_frame    = rospy.get_param("~root_frame", "torso_lift_link")
        self.controller_namespace   = rospy.get_param("~controller", "r_cart")

        self.do_viz      = True
        self.iters       = 0
        self.current_pose = None
        self.desired_pose  = None
        self.lookahead_pose = None
        self.controller_state = None
        self.controller_state_timer = Timer(10)

        # set up the visualization topic to show the desired point and the lookahead point
        self.viz_namespace = "/pose_follower"
        self.lookahead_pose_pub = rospy.Publisher(self.viz_namespace + "/lookahead_pose", Marker, queue_size = 1)
        
        # topic to send drive commands to
        self.control_pub = rospy.Publisher(self.controller_namespace + "/command_pose", PoseStamped, queue_size =1 )

        # topic to listen for desired_poses 
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        
        # topic to listen for odometry messages, either from particle filter or the simulator
        self.control_state_sub = rospy.Subscriber(self.controller_namespace+"/state", ControllerState, self.controller_state_callback, queue_size=1)


        rospy.loginfo("pure pursuit initialized! hello :)")

    def controller_state_callback(self, msg):

        self.controller_state = msg
        pos = msg.x.pose.position
        quat = msg.x.pose.orientation
        pt = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
        self.current_pose = np.array(pt)
        self.pose_follower( self.current_pose)
        # this is for timing info
        self.controller_state_timer.tick()
        if self.desired_pose is not None and self.iters % 20 == 0:
            rospy.loginfo("Control fps: %.2f"% self.controller_state_timer.fps())


    def visualize(self):
        ''' Publishes visualization topics:
        '''
        if not self.do_viz:
            return
        # visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
        if isinstance(self.lookahead_pose, np.ndarray):

            self.lookahead_pose_pub.publish(make_circle_marker(
                self.desired_pose, 0.015, [0.1,0.3,1.0], self.root_frame, self.viz_namespace+"desiredpose", 0, 3, .25))
            self.lookahead_pose_pub.publish(make_circle_marker(
                self.current_pose, self.lookahead*2, [0.0,0.0,1.0], self.root_frame, self.viz_namespace+"lookaheadsphere", 0, 3, .25))
            self.lookahead_pose_pub.publish(make_circle_marker(
                self.lookahead_pose, 0.015, [1.0,1.0,1.0], self.root_frame, self.viz_namespace+"lookahead", 1, 3))

    def pose_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        pos = msg.pose.position
        quat = msg.pose.orientation
        pt = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
        self.desired_pose = np.array(pt)
        msginfo =  ("Receiving new pose (%s)" % ",".join(["%.3f"%x for x in pt]))
        rospy.loginfo(msginfo)
        
    def compute_cartesian_lookahead(self, pose):
        x_desi = self.desired_pose[:3]
        x_curr = pose[:3]

        x_err = x_desi - x_curr
        x_err_norm = np.linalg.norm(x_err)

        if x_err_norm != 0:
            x_err = x_err/x_err_norm

        x_look = x_curr + x_err*self.lookahead

        return x_look, x_err_norm

    def compute_quaternion_lookahead(self, pose):
        x_desi = self.desired_pose[3:]
        x_curr = pose[3:]
        #slerp
        return x_curr, 0


     
    def pose_follower(self, pose):
        '''Determines and applies lookahead control law
        '''
        # stop if no trajectory has been received
        if self.desired_pose is None :
            return self.stop()

        # compute cartesian lookahead distance
        pos, cart_err = self.compute_cartesian_lookahead(pose)

        # compute quaternion lookahead
        quat, quat_err = self.compute_quaternion_lookahead(pose)
        
        self.lookahead_pose = np.hstack([pos,quat])

        # go to desired pose instead of lookahead if error is small
        if cart_err < self.lookahead:
            control_pose = pose
        else:
            control_pose = self.lookahead_pose

        self.iters += 1
        
        pose_msg=PoseStamped(make_header(self.root_frame), \
                Pose(Point(*control_pose[:3]), Quaternion(*control_pose[3:])))

        self.control_pub.publish(pose_msg)
        self.visualize()
       
    def stop(self):
        #rospy.loginfo("stop not implemented")
        return
        
def make_header(frame_id):
        return Header(0, rospy.Time(0), frame_id)

def make_arrow_marker(point, scale, color, frame_id, namespace, sid, duration=20):
    marker = Marker()
    marker.header = make_header(frame_id)
    marker.ns = namespace
    marker.id = sid
    marker.type = 0
    marker.lifetime = rospy.Duration.from_sec(duration)
    marker.action = 0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = point[2]
    marker.pose.orientation.w = point[3]
    marker.pose.orientation.x = point[4]
    marker.pose.orientation.y = point[5]
    marker.pose.orientation.z = point[6]
    marker.scale.x = scale
    marker.scale.y = .1*scale
    marker.scale.z = .1*scale
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = .5
    return marker



def make_circle_marker(point, scale, color, frame_id, namespace, sid, duration=20, alpha = .5):
    marker = Marker()
    marker.header = make_header(frame_id)
    marker.ns = namespace
    marker.id = sid
    marker.type = 2
    marker.lifetime = rospy.Duration.from_sec(duration)
    marker.action = 0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = point[2]
    marker.pose.orientation.w = point[3]
    marker.pose.orientation.x = point[4]
    marker.pose.orientation.y = point[5]
    marker.pose.orientation.z = point[6]
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = alpha
    return marker

class CircularArray(object):
    """docstring for CircularArray"""
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])

class Timer:
    def __init__(self, smoothing):
        self.arr = CircularArray(smoothing)
        self.last_time = rospy.Time.now().to_sec()

    def tick(self):
        t = rospy.Time.now().to_sec()
        if (t - self.last_time) == 0:
            return
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    def fps(self):

        return self.arr.mean()



def pt_in_segment(p1, p2, pt):
    return (np.all(pt >= p1) and np.all(pt <= p2)) or\
            (np.all(pt >= p2) and np.all(pt<= p1))
    
"""      
def dist_line_pt(p1, p2, pt):
    x0,y0 = pt
    x1,y1 = p1
    x2,y2 = p2

    num =abs((y2 - y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
    den =( (y2-y1)**2 + (x2-x1)**2)** 0.5
    if den != 0:
        d = num/den
    else:
        d = ( (x0 - x1)**2 + (y0-y1)**2 )**0.5

    # make sure pt is on line segment
    n_l = [-(y2-y1), (x2-x1)]
    n_r = [(y2-y1), -(x2-x1)]
    valids = []
    for n in [n_l, n_r]:
        n_hat = MU.unit(n)
        x_i = x0 + n_hat[0]*abs(d)
        y_i = y0 + n_hat[1]*abs(d)
        p_i = (x_i, y_i)

        valids.append(pt_in_segment(p1, p2, p_i) )# (in_range(x1, x2, x_i) and in_range(y1, y2, y_i))

    if not any(valids): 
        d1 = MU.norm(pt, p1)
        d2 = MU.norm(pt, p2)
        d  = d1 if d1 <= d2 else d2
    #print '\t', p1,p2,pt,
    #print "\t %s" % any(valids), d
    return d


def dist_line_segs((a1,a2), (b1,b2) ):
    dists = []
    dists.append(dist_line_pt (a1, a2, b1))
    dists.append(dist_line_pt (a1, a2, b2))
    dists.append(dist_line_pt (b1, b2, a1))
    dists.append(dist_line_pt (b1, b2, a2))
    return min(dists)
"""

if __name__=="__main__":
    rospy.init_node("pose_follower")
    pf = PoseFollower()
    rospy.spin()
