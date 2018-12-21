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

class PurePursuit:
    path = []
    path_set = False
    
    def __init__(self, tf_listener=None):
        """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
            Modified from Corey Walsh's implementation for the MIT Racecar.
            Set point determined with the method described here: 
            http://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2006_1/howard_thomas_2006_1.pdf
        """
        self.trajectory_topic = rospy.get_param("~trajectory_topic", "path_generated")
        self.lookahead        = float(rospy.get_param("~lookahead", ".02"))
        self.max_reacquire    = rospy.get_param("~max_reacquire", ".1")
        self.root_frame    = rospy.get_param("~root_frame", "torso_lift_link")
        self.speed            = float(rospy.get_param("~speed", "0.1"))
        self.wrap             = bool(rospy.get_param("~wrap", "True"))
        wheelbase_length      = float(rospy.get_param("~wheelbase", "1.0"))
        self.controller_namespace   = rospy.get_param("~controller", "r_cart")

        self.do_viz      = True
        self.iters       = 0
        self.nearest_point   = None
        self.lookahead_point = None
        self.controller_state = None
        self.current_segment = 0
        self.controller_state_timer = Timer(10)
        self.trajectory = None
        self.on_trajectory = False 

        # set up the visualization topic to show the nearest point on the trajectory, and the lookahead point
        self.viz_namespace = "/pure_pursuit"
        self.nearest_point_pub = rospy.Publisher(self.viz_namespace + "/nearest_point", Marker, queue_size = 1)
        self.lookahead_point_pub = rospy.Publisher(self.viz_namespace + "/lookahead_point", Marker, queue_size = 1)
        
        # topic to send drive commands to
        self.control_pub = rospy.Publisher(self.controller_namespace + "/command_pose", PoseStamped, queue_size =1 )
        #self.control_pub = rospy.Publisher(self.drive_topic, PoseStamped, queue_size =1 )

        # topic to listen for trajectories
        self.traj_sub = rospy.Subscriber(self.trajectory_topic, Path, self.trajectory_callback, queue_size=1)
        
        # topic to listen for odometry messages, either from particle filter or the simulator
        self.control_state_sub = rospy.Subscriber(self.controller_namespace+"/state", ControllerState, self.controller_state_callback, queue_size=1)


        rospy.loginfo("pure pursuit initialized! hello :)")

    def controller_state_callback(self, msg):

        self.controller_state = msg
        pos = msg.x.pose.position
        quat = msg.x.pose.orientation
        pt = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
        self.pose = np.array(pt)
        self.pure_pursuit( self.pose)
        # this is for timing info
        self.controller_state_timer.tick()
        if self.trajectory is not None and self.iters % 20 == 0:
            rospy.loginfo("Control fps: %.2f"% self.controller_state_timer.fps())


    def visualize(self):
        ''' Publishes visualization topics:
               - Circle to indicate the nearest point along the trajectory
               - Circle to indicate the chosen lookahead point
        '''
        if not self.do_viz:
            return
        # visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
        if self.nearest_point_pub.get_num_connections() > 0 \
                and isinstance(self.nearest_point, np.ndarray):
            self.nearest_point_pub.publish(make_circle_marker(
                self.nearest_point, 0.005, [0.1,0.3,1.0], self.root_frame, self.viz_namespace+"nearestontraj", 0, 3, .25))
            self.nearest_point_pub.publish(make_circle_marker(
                self.pose, self.lookahead*2, [0.0,0.0,1.0], self.root_frame, self.viz_namespace+"currentpose", 0, 3, .25))

        if self.lookahead_point_pub.get_num_connections() > 0 \
                and isinstance(self.lookahead_point, np.ndarray):
            self.lookahead_point_pub.publish(make_circle_marker(
                self.lookahead_point, 0.005, [1.0,1.0,1.0], self.root_frame, self.viz_namespace, 1, 3))

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        msginfo =  "Receiving new trajectory:" + str( len(msg.poses)) +  "points" 
        rospy.loginfo(msginfo)
        self.trajectory = None
        self.current_segment = 0
        rospy.loginfo("HACK! waiting to go to start of traj.")
        rospy.sleep(3)
        traj = [self.pose]
        for pose in msg.poses:
            pos = pose.pose.position
            quat = pose.pose.orientation
            pt = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
            traj.append(pt)
        self.trajectory = np.array(traj)
        
    def pt_to_line_segment_distance(self, pt, p1, p2, onpath=False):
        ''' returns normal vector and distance
        pt, p1, and p2 are 3x1 vectors
        '''
        # the line is defined as p1 + t*(p2-p1)
        # line = p1 + v*t where v = p2-p1

        # vector representing line 
        v = p2 - p1

        # vector from pt to p1
        u = p1 - pt

        u_mag = np.linalg.norm(u)
        v_mag = np.linalg.norm(v)
        
        vhat = v if v_mag == 0 else v/v_mag
        uhat = u if u_mag == 0 else u/u_mag      

        
        # perp vector
        n = u - np.dot(u, vhat)*vhat

        n_mag = np.linalg.norm(n)

        # project the point onto the line 
        pt_proj = pt + n 
        
        
        perp_dist = n_mag
        
        nhat = n if n_mag == 0 else n/n_mag
        if pt_in_segment(p1, p2, pt_proj):
            # if the projected point is on the line segment
            # then the distance is  the perpindicular distance
            dist = perp_dist
        else: 
            # if the point is not on the line segment then the distance is
            # the minimum distance to the end points.
            d1 =u_mag

            #vector from pt to p2
            w = p2 - pt
            w_mag = np.linalg.norm(w)
            d2 = w_mag
            what = w if w_mag == 0 else w/w_mag

            # update normal vector to point toward the end points
            if d1 < d2 and not onpath:
                nhat = uhat 
                dist = d1
                pt_proj = p1
            else:
                nhat = what
                dist = d2
                pt_proj = p2
        return dist, nhat, pt_proj, vhat

    def compute_lookahead_segment(self, pt, p1, p2):

        dist, nhat, nearest_pt, vhat = self.pt_to_line_segment_distance(pt, p1, p2)


        # compute lookahead point and compute its distance to trajectory
        lookahead_pt = None
        dist_to_traj = 0
        
        if dist == self.lookahead:
            
            # lookahead point is tangent to the circle
            # we have already computed this point
            lookahead_pt = nearest_pt

        elif dist < self.lookahead:
            # find the two points that intersect the circle and line

            # the intersection points are a right triangle
            # from the center to the projected pt is the perp. distance
            # the hypotenuse is the lookahead distance
            # the length we should move down the line is defined by pytheagorean theorem
            opp = np.sqrt(self.lookahead**2 - dist**2)
            
            # v is the direction of the line
            #intersect1 = nearest_pt + opp*v 
            #intersect2 = nearest_pt - opp*v 
            # always use further point
            lookahead_pt = nearest_pt + opp*vhat

            # make sure lookahead point is defined on segment
            if not pt_in_segment(p1, p2, lookahead_pt):
                # lookeahed point is too far, just use end of line segment
                lookahead_pt = p2
                dist_from_traj = 0

        else:
            # project lookahead distance from current point along the normal
            lookahead_pt = pt + self.lookahead*nhat
            dist_to_traj = dist - self.lookahead
        return (dist_to_traj, lookahead_pt), (dist, nearest_pt)


    def lookahead_point_on_trajectory(self, point):
        lookahead_dists = []
        nearest_dists = []

        #self.control_state_sub.unregister()

        for i in range(len(self.trajectory)):
            p1 = self.trajectory[i-1, (0,1,2)]
            p2 = self.trajectory[i, (0,1,2)]

            (dist_to_traj, lookahead_pt), (dist, nearest_pt) = \
                    self.compute_lookahead_segment(point, p1, p2)
            #print dist_to_traj, dist

            nearest_pose = np.array(nearest_pt.tolist() + self.trajectory[i,3:].tolist()) 
            self.nearest_point_pub.publish(make_circle_marker(
                nearest_pose, 0.01, [1.0,1.0,.0], self.root_frame, self.viz_namespace+"baa" + str(i), 1, 3))

            lookahead_pose = np.array(lookahead_pt.tolist() + self.trajectory[i,3:].tolist()) 
            self.lookahead_point_pub.publish(make_circle_marker(
                lookahead_pose, 0.01, [0.0,1.0,1.0], self.root_frame, self.viz_namespace+"baa" + str(i), 1, 3))

            nearest_dists.append ( (dist, nearest_pose, i))
            lookahead_dists.append( (dist_to_traj, lookahead_pose))
        #import pdb; pdb.set_trace()
        lookahead_dists = sorted(lookahead_dists, key=lambda x: x[0])
        lookahead_pose = lookahead_dists[0][1]
        return nearest_pose, lookahead_pose


        

    """

    def lookahead_point_on_trajectory(self, point):
        dists = []
        for i in range(len(self.trajectory)):
            p1 = self.trajectory[i-1, (0,1,2)]
            p2 = self.trajectory[i, (0,1,2)]
            d, n, pt, v = self.pt_to_line_segment_distance(point, p1, p2)
            d = np.round(d,5)
            dists.append((d,n,pt,v,i))
            
            pose = np.array(pt.tolist() + self.trajectory[i,3:].tolist()) 
            self.lookahead_point_pub.publish(make_circle_marker(
                pose, 0.001, [1.0,0.0,.0], self.root_frame, self.viz_namespace+"baa" + str(i), 1, 3))


        nearest = min(dists, key=lambda x: x[0])
        same_dists = [x for x in dists if x[0] == nearest[0]]
        if len(same_dists) > 1:
            nearest= min(same_dists, key=lambda x: self.current_segment - x[-1])
        d, n, nearest_pt, v, nearest_i = nearest
        
        if np.linalg.norm(nearest_pt - point) < self.lookahead:
            on_trajectory = False
        else:
            on_trajectory = True

        # recompute nearest_pt for current line segment.  force
        # always moving forward on path
        p1 = self.trajectory[nearest_i-1, (0,1,2)]
        p2 = self.trajectory[nearest_i, (0,1,2)]
        d, n, pt, v = self.pt_to_line_segment_distance(point, p1, p2,onpath=True)
        
        # ignore earlier segments
        dists = dists[nearest_i:]
        dists[0] = (d,n,pt,v,nearest_i)
        self.current_segment = nearest_i

        on_trajectory = np.linalg.norm(pt - point) < self.lookahead

        lookahead_points = []
        for (d, n, pt, v, j) in  dists:


            if d == self.lookahead:
                # lookahead point is tangent to the circle
                # we have already computed this point
                lookahead_pt = pt
                dist_from_traj = 0

            elif d < self.lookahead:
                # find the two points that intersect the circle and line
                
                # the intersection points are a right triangle
                # from the center to the projected pt is the perp. distance
                # the hypotenuse is the lookahead distance
                # the length we should move down the line is defined by pytheagorean theorem
                opp = np.sqrt(self.lookahead**2 - d**2)
                
                # v is the direction of the line
                #intersect1 = pt + opp*v 
                #intersect2 = pt - opp*v 
                # always use further point?
                lookahead_pt = pt + opp*v 

                # make sure lookahead point is defined on segment
                p1 = self.trajectory[j-1, (0,1,2)]
                p2 = self.trajectory[j, (0,1,2)]
                dist_from_traj = 0

                if not pt_in_segment(p1, p2, lookahead_pt):
                    # lookeahed point is too far, just use end of line segment
                    lookahead_pt = p2
                    dist_from_traj = 0
            else:
                # project lookahead distance from point along the normal
                lookahead_pt = point + self.lookahead*n
                dist_from_traj = np.round(np.abs(d - self.lookahead),5)

            d_to_lookahead = np.linalg.norm(point - lookahead_pt)
            dist_from_lookahead = np.round(np.abs(self.lookahead - d_to_lookahead), 5)
            
            dist_from_current_segment = j - self.current_segment  
            
            lookahead_points.append((dist_from_lookahead,dist_from_traj, d_to_lookahead,dist_from_current_segment,  lookahead_pt, j))
            #lookahead_points.append((dist_from_lookahead,dist_from_traj, dist_from_current_segment,  lookahead_pt, j))
        try:

            dist_from_lookahead, dist_from_traj, d_to_lookahead, dist_from_current_segment, lookahead_pt, i = min(lookahead_points, key = lambda x:x[0:4])
            lookahead_points = sorted(lookahead_points, key = lambda x:x[0:4])
            
            nearest = lookahead_points[0][0:3]
            same_dists = [x for x in lookahead_points if x[0:3] ==nearest]

            #if len(same_dists) > 1:
            #    dist_from_lookahead, dist_from_traj, d_to_lookahead, dist_from_current_segment, lookahead_pt, i  = same_dists[-1]

        except Exception as e:
            print e
            self.trajectory = None
            import pdb; pdb.set_trace()

        # just use quaternion from traj for now 
        # quaternion should be interpolated tho :(
        lookahead_pt =np.array( lookahead_pt.tolist() + self.trajectory[i, 3:].tolist())
        nearest_pt = np.array (nearest_pt.tolist() +  self.trajectory[i, 3:].tolist())
        
        for j, traj in enumerate(self.trajectory[i:]):
            pose = lookahead_points[j][4].tolist() + traj[3:].tolist()
            
            #self.lookahead_point_pub.publish(make_circle_marker(
            #    pose, 0.001, [0.0,1.0,1.0], self.root_frame, self.viz_namespace+"aa" + str(j), 1, 3))

        return nearest_pt, lookahead_pt
    """


    def nearest_point_on_trajectory(self, point):
        ''' return the closet point based on cartesian distance only'''
        dist = np.linalg.norm(self.trajectory[:,(0,1,2)] - point, axis=1)
        dist_i = np.argmin(dist)
        return self.trajectory[dist_i], dist_i

    def pure_pursuit(self, pose):
        ''' Determines and applies Pure Pursuit control law

            1. Find the nearest point on the trajectory
            2. Traverse the trajectory looking for the nearest point that is the lookahead distance away from the 
               car, and further along the path than the nearest point from step (1). This is the lookahead point.
            3. Determine steering angle necessary to travel to the lookahead point from step (2)
            4. Send the desired speed and steering angle commands to the robot

            Special cases:
                - If nearest_point is beyond the max path reacquisition distance, stop
                - If nearest_point is between max reacquisition dist and lookahead dist, navigate to nearest_point
                - If nearest_point is less than the lookahead distance, find the lookahead point as normal
        '''
        # stop if no trajectory has been received
        if self.trajectory is None :#
            return self.stop()
        
        self.iters += 1

        self.nearest_point, self.lookahead_point = self.lookahead_point_on_trajectory(pose[:3])

        pose =PoseStamped(make_header(self.root_frame),  Pose(Point(*self.lookahead_point[:3]), Quaternion(*self.lookahead_point[3:])))
        if (self.iters %1) == 0:
            self.control_pub.publish(pose)
        
        self.visualize()
       
    def stop(self):
        #rospy.loginfo("stop not implemented")
        return
        
def make_header(frame_id):
        return Header(0, rospy.Time(0), frame_id)

def make_arrow_marker(point, scale, color, frame_id, namespace, sid, duration=0):
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



def make_circle_marker(point, scale, color, frame_id, namespace, sid, duration=0, alpha = .5):
    marker = Marker()
    marker.header = make_header(frame_id)
    marker.ns = namespace
    marker.id = sid
    marker.type = 2
    marker.lifetime = rospy.Duration.from_sec(20)#duration)
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


def in_range(x1, x2, x0):
    if x1 < x2:
        a = x1
        b = x2
    else:
        a = x2
        b = x1
    left = np.round(x0 - a,5)
    right = np.round(b-x0, 5)
    return left >= 0 and right >= 0

def pt_in_segment(p1, p2, pt):
    return (np.all(pt >= p1) and np.all(pt <= p2)) or\
            (np.all(pt >= p2) and np.all(pt<= p1))
    valid =  True
    for i in range(3):
        valid &= in_range(p1[i], p2[i], pt[i])
    return valid

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
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()

