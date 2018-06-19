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
from robot_mechanism_controllers import JTCartesianControllerState\
        as ControllerState


class PurePursuit:
    path = []
    path_set = False
    
    def __init__(self, tf_listener=None):
        """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
            Modified from Corey Walsh's implementation for the MIT Racecar.
            Set point determined with the method described here: 
            http://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2006_1/howard_thomas_2006_1.pdf
        """
        self.trajectory_topic = rospy.get_param("~trajectory_topic")
        self.lookahead        = rospy.get_param("~lookahead")
        self.max_reacquire    = rospy.get_param("~max_reacquire")
        self.root_frame    = rospy.get_param("~root_frame")
        self.speed            = float(rospy.get_param("~speed"))
        self.wrap             = bool(rospy.get_param("~wrap"))
        wheelbase_length      = float(rospy.get_param("~wheelbase"))
        self.controller_namespace   = rospy.get_param("~l_cart")

        self.do_viz      = True
        self.iters       = 0
        self.nearest_point   = None
        self.lookahead_point = None
        self.controller_state = None
        self.controller_state_timer = Timer(10)

        # set up the visualization topic to show the nearest point on the trajectory, and the lookahead point
        self.viz_namespace = "/pure_pursuit"
        self.nearest_point_pub = rospy.Publisher(self.viz_namespace + "/nearest_point", Marker, queue_size = 1)
        self.lookahead_point_pub = rospy.Publisher(self.viz_namespace + "/lookahead_point", Marker, queue_size = 1)
        
        # topic to send drive commands to
        self.control_pub = rospy.Publisher(self.drive_topic, PoseStamped, queue_size =1 )

        # topic to listen for trajectories
        self.traj_sub = rospy.Subscriber(self.trajectory_topic, Path, self.trajectory_callback, queue_size=1)
        
        # topic to listen for odometry messages, either from particle filter or the simulator
        self.control_state_sub = rospy.Subscriber(self.controller_namespace+"/state", ControllerState, self.controller_state_callback, queue_size=1)


        rospy.loginfo("pure pursuit initialized! hello :)")

    def controller_state_callback(self, msg):
        self.controller_state = msg
        self.pure_pursuit(msg.x.pose)
        # this is for timing info
        self.controller_state_timer.tick()
        self.iters += 1
        if self.iters % 20 == 0:
            rospy.loginfo("Control fps:", self.controller_state_timer.fps())


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
                self.nearest_point, 0.5, [0.0,0.0,1.0], self.root_frame, self.viz_namespace, 0, 3))

        if self.lookahead_point_pub.get_num_connections() > 0 \
                and isinstance(self.lookahead_point, np.ndarray):
            self.lookahead_point_pub.publish(umake_circle_marker(
                self.lookahead_point, 0.5, [1.0,1.0,1.0], self.root_frame, self.viz_namespace, 1, 3))

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        msg =  "Receiving new trajectory:" + str( len(msg.poses)) +  "points" 
        rospy.loginfo(msg)
        traj = []
        for pose in msg.poses:
            pos = pose.position
            quat = pose.quaternion
            pt = [pos.x, pos.y, pos.z, quat.w, quat.x, quat.y, quat.z]
            traj.append( np.array(traj))
        self.trajectory = np.array(traj)
        
        #self.trajectory.clear()
        #fromPolygon(msg.polygon)
        #self.trajectory.fromPolygon(msg.polygon)
        #self.trajectory.publish_viz(duration=0.0)

    def nearest_point_on_trajectory(self, point):
        ''' return the closet point based on cartesian distance only'''

        dist = self.traj[:,(0,1,2)] - point
        dist_i = np.argmin(dist)
        return self.traj[dist_i]

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
        if self.trajectory.empty():
            return self.stop()

        # this instructs the trajectory to convert the list of waypoints into a numpy matrix
        if self.trajectory.dirty():
            self.trajectory.make_np_array()

        # step 1
        nearest_point, nearest_dist, t, i = utils.nearest_point_on_trajectory(pose[:2], self.trajectory.np_points)
        self.nearest_point = nearest_point

        if nearest_dist < self.lookahead:
            # step 2
            lookahead_point, i2, t2 = \
                utils.first_point_on_trajectory_intersecting_circle(pose[:2], self.lookahead, self.trajectory.np_points, i+t, wrap=self.wrap)
            if i2 == None:
                if self.iters % 5 == 0:
                    print "Could not find intersection, end of path?"
                self.lookahead_point = None
            else:
                if self.iters % 5 == 0:
                    print "found lookahead point"
                self.lookahead_point = lookahead_point
        elif nearest_dist < self.max_reacquire:
            if self.iters % 5 == 0:
                print "Reacquiring trajectory"
            self.lookahead_point = self.nearest_point
        else:
            self.lookahead_point = None

        # stop of there is no navigation target, otherwise use ackermann geometry to navigate there
        if not isinstance(self.lookahead_point, np.ndarray):
            self.stop()
        else:
            steering_angle = self.determine_steering_angle(pose, self.lookahead_point)
            # send the control commands
            self.apply_control(self.speed, steering_angle)

        self.visualize()

    def determine_steering_angle(self, pose, lookahead_point):
        ''' Given a robot pose, and a lookahead point, determine the open loop control 
            necessary to navigate to that lookahead point. Uses Ackermann steering geometry.
        '''
        # get the lookahead point in the coordinate frame of the car
        rot = utils.rotation_matrix(-pose[2])
        delta = np.array([lookahead_point - pose[0:2]]).transpose()
        local_delta = (rot*delta).transpose()
        local_delta = np.array([local_delta[0,0], local_delta[0,1]])
        # use the ackermann model
        steering_angle = self.model.steering_angle(local_delta)
        return steering_angle
        
    def apply_control(self, speed, steering_angle):
        self.actual_speed = speed
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        drive_msg.speed = speed
        drive_msg.steering_angle = steering_angle
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
        self.control_pub.publish(drive_msg_stamped)

    def stop(self):
        print "Stopping"
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        drive_msg.speed = 0
        drive_msg.steering_angle = 0
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
        self.control_pub.publish(drive_msg_stamped)
def make_circle_marker(point, scale, color, frame_id, namespace, sid, duration=0):
    marker = Marker()
    marker.header = make_header(frame_id)
    marker.ns = namespace
    marker.id = sid
    marker.type = 2 # sphere
    marker.lifetime = rospy.Duration.from_sec(duration)
    marker.action = 0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = 1.0
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
        self.last_time = time.time()

    def tick(self):
        t = time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    def fps(self):
        return self.arr.mean()
if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
