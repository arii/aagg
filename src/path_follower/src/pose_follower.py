#!/usr/bin/env python

from geometry_msgs.msg import Pose, PoseStamped, Point, \
                              Quaternion, PointStamped
from nav_msgs.msg import Path

from std_msgs.msg import Header
import numpy as np
import tf
import rospy
import actionlib
import path_follower.msg 
import tf.transformations as tfx


from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA
from robot_mechanism_controllers.msg import JTCartesianControllerState \
        as ControllerState

"""
TODO:   Test on robot


"""

class PoseFollower(object):
    _feedback = path_follower.msg.GagaPoseFeedback()
    _result = path_follower.msg.GagaPoseResult()
    
    def __init__(self):
        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(
                self._action_name, 
                path_follower.msg.GagaPoseAction,
                execute_cb = self.execute_cb,
                auto_start = False
                )


        self.pose_topic = rospy.get_param("~pose_topic", "pose_generated")
        self.lookahead        = float(rospy.get_param("~lookahead", ".01"))
        self.lookahead_quat        = float(rospy.get_param("~lookaheadquat", "0.1"))
        self.cart_stopped = float(rospy.get_param("~cartstopped", "0.001"))
        self.ang_stopped = float(rospy.get_param("~angstopped", "0.0001"))
        self.goal_cartesian_tolerance       = float(rospy.get_param("~goal_cartesian_tolerance", ".01"))
        self.goal_quaternion_tolerance       = float(rospy.get_param("~goal_quaternion_tolerance", ".01"))
        self.max_reacquire    = rospy.get_param("~max_reacquire", ".1")
        self.root_frame    = rospy.get_param("~root_frame", "torso_lift_link")
        self.controller_namespace   = rospy.get_param("~controller", "r_cart")

        self.do_viz      = True
        self.iters       = 0
        self.current_pose = None
        self.desired_pose  = None
        self.lookahead_pose = None
        self.controller_state = None
        self.abort = False
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
        
        self._as.start()

        rospy.loginfo("pure pursuit initialized! hello :)")

    def execute_cb(self, goal):
        if goal.lookahead != 0 and self.lookahead != goal.lookahead:
            self.lookahead = goal.lookahead
            rospy.loginfo ("updated lookahead to %.3f" % goal.lookahead)
        
        if goal.goal_cartesian_tolerance != 0 and self.goal_cartesian_tolerance != goal.goal_cartesian_tolerance:
            self.goal_cartesian_tolerance = goal.goal_cartesian_tolerance
            rospy.loginfo ("updated goal_cartesian_tolerance to %.3f" % goal.goal_cartesian_tolerance)
        
        if goal.goal_quaternion_tolerance != 0 and self.goal_quaternion_tolerance != goal.goal_quaternion_tolerance:
            self.goal_quaternion_tolerance = goal.goal_quaternion_tolerance
            rospy.loginfo ("updated goal_quaternion_tolerance to %.3f" % goal.goal_quaternion_tolerance)

        if goal.timeout == rospy.Duration(0):
            rospy.loginfo("no timeout requested")
            endtime = None
        else:
            endtime = rospy.Time.now() + goal.timeout
        
        self.iters = 0
        self.pose_callback(goal.pose)

        alerted = False
        r = rospy.Rate(100)
        while self._as.is_active():
            if endtime is not None and rospy.Time.now() > endtime:
                if not alerted:
                    rospy.loginfo("Ran out of time!")
                    self.abort = True
            r.sleep()



    def controller_state_callback(self, msg):

        self.controller_state = msg
        pos = msg.x.pose.position
        quat = msg.x.pose.orientation
        pt = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
        self.current_pose = np.array(pt)
        self.pose_follower( self.current_pose)


    def visualize(self):
        ''' Publishes visualization topics:
        '''
        if not self.do_viz:
            return
        # visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
        if isinstance(self.lookahead_pose, np.ndarray):

            self.lookahead_pose_pub.publish(make_circle_marker(
                self.desired_pose, 0.015, [0.,1.0,.0], self.root_frame, self.viz_namespace+"desiredpose", 0, 3, .25))
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
        self.abort = False
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

        quat = tfx.quaternion_slerp(x_curr, x_desi, self.lookahead_quat)
        # compute angle between quaternions
        err = np.abs(np.arccos(np.dot(x_desi, x_curr))) % np.pi
    
        return quat, err #x_curr, 0.0

    def detect_stopped(self):
        twist = self.controller_state.xd

        lin = [twist.linear.x, twist.linear.y, twist.linear.z]
        ang = [twist.angular.x, twist.angular.y, twist.angular.z]
        cart_motion = np.linalg.norm(lin)
        ang_motion = np.linalg.norm(ang)
        return cart_motion, ang_motion
        

     
    def pose_follower(self, pose):
        '''Determines and applies lookahead control law
        '''
        
        # stop if no trajectory has been received
        if self.desired_pose is None  or not self._as.is_active():
            return self.stop()

        if self._as.is_preempt_requested():
            self.desired_pose = None
            self._as.set_preempted()
            return self.stop()

        if self.abort:
            self.desired_pose = None
            self._as.set_aborted()
            return self.stop()
        


        # compute cartesian lookahead distance
        pos, cart_error = self.compute_cartesian_lookahead(pose)

        # compute quaternion lookahead
        quat, quat_error = self.compute_quaternion_lookahead(pose)
        
        self.lookahead_pose = np.hstack([pos,quat])

        # go to desired pose instead of lookahead if error is small
        if cart_error < self.lookahead:
            control_pos = pose[:3]
        else:
            control_pos = pos

        if quat_error < self.lookahead_quat:
            control_quat = pose[3:]
        else:
            control_quat = quat

        control_pose = np.hstack([control_pos, control_quat])


        self.iters += 1
        
        pose_msg=PoseStamped(make_header(self.root_frame), \
                Pose(Point(*control_pose[:3]), Quaternion(*control_pose[3:])))

        self.control_pub.publish(pose_msg)
        self.controller_state_timer.tick()
        


        if self.iters  % 50 == 0:
            self._feedback.cartesian_error = cart_error
            self._feedback.quaternion_error = quat_error
            self._feedback.fps = self.controller_state_timer.fps()
            self._as.publish_feedback(self._feedback)
            self.visualize()
   
        if cart_error <= self.goal_cartesian_tolerance and \
            quat_error <= self.goal_quaternion_tolerance:
            rospy.loginfo("Succeeded in reaching desired pose")
            self._as.set_succeeded(self._result)

        else:
            if self.iters < 10: 
                return
            cart_motion, ang_motion = self.detect_stopped()
            if cart_motion < self.cart_stopped and ang_motion < self.ang_stopped:
                rospy.logwarn("Stop detected and goal position not reached.  Abort!")
                self._as.set_aborted()


       
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
    

if __name__=="__main__":
    rospy.init_node("pose_follower")
    pf = PoseFollower()
    rospy.spin()
