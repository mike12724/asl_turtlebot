#!/usr/bin/env python

"""

TODO: Integrate LIDAR data into our extract_location function!
Actually detect fruits; fix supervisor to detect all the things in the coco file


"""

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Header
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Point, PointStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
import numpy as np
from enum import Enum
import detector # for loading object labels
from collections import defaultdict 
from utils import THETA_EPS, POS_EPS

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
#POS_EPS = .1
#THETA_EPS = np.pi / 4.0

#threshold for considering two detected objects as the same
DETECT_EPS = 0.3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = 0.7

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "rviz = %s\n" % rviz
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        # initialize variables

        #Each entry in map is a list of tuples of locations of objects of that type
        #map[name] = [(obj_1_x,obj_1_y), (obj_2_x, obj_2_y), (obj_3)...]
        self.map = defaultdict(list) 
        self.x = 0
        self.y = 0
        self.theta = 0
        self.V = 0
        self.x_g, self.y_g, self.theta_g = 0.0, 0.0, 0.0
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.stop_sign_loc = None #for checking if we've successfully crossed
        self.delivery_targets = [] #list of objects we need to nav to
        self.trans_listener = tf.TransformListener()
        # command pose for controller






        #publish to cmd_nav instead to use navigator.py!!!!
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # publish to obj_loc_broadcaster
        self.obj_loc_publisher = rospy.Publisher('/supervisor/object_position', PoseStamped, queue_size=10)


        # subscribers
        # stop sign detector
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        rospy.Subscriber('/delivery_request', String, self.delivery_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # if using rviz, we can subscribe to nav goal click
        if rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        
        # Need to load the list of all object labels
        self.dict_of_all_obj_labels = detector.load_object_labels(detector.PATH_TO_LABELS)

        self.detected_object = None
        for class_label in self.dict_of_all_obj_labels.values():
            # Here, we can pick which labels we want a subscriber for. 
            # Check out the COCO dataset if you want to know which are options.
            #if not class_label in ["stop_sign", "banana"]: continue
            if not class_label in ["person", "banana", "bottle", "backpack", "stop_sign"]: continue
            print("INFO: creating subscriber for: {}".format(class_label))
            rospy.Subscriber(
                '/detector/'+class_label, 
                DetectedObject, 
                self.extract_object_location)

    def vel_callback(self,msg):
        self.V = msg.linear.x
    
    def extract_object_location(self,msg):
        th_r, th_l     = msg.thetaright, msg.thetaleft

        # Switch from [0,2pi] wrap to [-pi, pi]
        if th_r > math.pi:
            th_r = th_r - 2*math.pi
        if th_l > math.pi:
            th_l = th_l - 2*math.pi
        th_c = 0.5*(th_r + th_l) # THIS IS AN APPROX

        dist = msg.distance
        x_from_cam = dist * math.cos(th_c)
        y_from_cam = dist * math.sin(th_c)
        theta_from_cam = math.pi+self.theta #assume sign is directly facing us

        if dist > 0.0:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time(0)

            #detected relative to robot, Also need msg.name to figure out what type of object it is
            pose.header.frame_id = '/base_footprint/'+msg.name 
            pose.pose.position.x = x_from_cam
            pose.pose.position.y = y_from_cam
            pose.pose.position.z = 0.0

            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta_from_cam)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            self.obj_loc_publisher.publish(pose)

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:
            
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        if self.mode == Mode.IDLE:
            self.mode = Mode.NAV

    def delivery_callback(self,msg):
        rospy.loginfo('Delivery received, splitting data!')
        delivery_targets = msg.data.split(',')
        if len(self.delivery_targets) == 0:
            #self.delivery_targets = ['/'+target+'_0' for target in delivery_targets]
            self.delivery_targets = [target for target in delivery_targets]

    def get_next_target(self):
        rospy.loginfo(("Getting next target, current list:", self.delivery_targets))
        if 'return_to_home' in self.delivery_targets[0]:
            self.x_g = 0.0
            self.y_g = 0.0
            self.theta_g = 0.0
            return

        tgts_plus_dist = []
        while len(self.delivery_targets) > 0: #danger danger, while loop
            obj = self.delivery_targets[0]
            try:
                (dist_1,rot) = self.trans_listener.lookupTransform(obj, '/base_footprint', rospy.Time()) 
                tgts_plus_dist.append((obj, math.sqrt(dist_1[0]**2 + dist_1[1]**2)))
                del self.delivery_targets[0]

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                #rospy.loginfo(e)
                pass

        self.delivery_targets = sorted(tgts_plus_dist, key=lambda tup: tup[1])
        self.delivery_targets = [a[0] for a in self.delivery_targets] 
        self.delivery_targets.reverse() #just need names (NOTE IN REVERSED ORDER!!! Used for popping)

        self.set_next_target(self.delivery_targets.pop())

    def set_next_target(self, tgt_name):
        rospy.loginfo(("Setting destination:", tgt_name))
        while True:
            try:
                (dist_1,rot) = self.trans_listener.lookupTransform('/map', tgt_name, rospy.Time()) 
                ang = np.arctan2(dist_1[1]-self.y, dist_1[0]-self.x)             
                self.x_g = dist_1[0]
                self.y_g = dist_1[1]
                self.theta_g = ang
                break

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo(e)
        if self.mode == Mode.IDLE:
            self.mode = Mode.NAV
        rospy.loginfo("Destination set!")

    def within_range_of_stop(self, sign_frame_id):
        try:
            t = self.trans_listener.getLatestCommonTime(sign_frame_id, '/base_footprint')
            #self.trans_listener.waitForTransform(sign_frame_id, '/base_footprint', rospy.Time().now(), rospy.Duration(4.0))
            (dist_1,rot) = self.trans_listener.lookupTransform(sign_frame_id, '/base_footprint', rospy.Time()) 
            euler = tf.transformations.euler_from_quaternion(rot)
            th = euler[2]
            if dist_1[0] > 0 and dist_1[0] < STOP_MIN_DIST and abs(dist_1[1]) < STOP_MIN_DIST and abs(th) >= 3*np.pi/4:
                return True
            else:
                return False

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            #rospy.loginfo(e)
            return -1


    def detect_stop(self, sign_frame_id):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        win = self.within_range_of_stop(sign_frame_id)
        if win == -1: return

        if win and self.mode == Mode.NAV: #if in front of sign, and closer than 1 second ago (by some error margin)
                self.init_stop_sign(sign_frame_id)


    ############ your code starts here ############
    # feel free to change the code here 
    # you may or may not find these functions useful
    # there is no "one answer"


    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        #self.pose_goal_publisher.publish(nav_g_msg)
        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self, sign_frame_id):
        """ initiates a stop sign maneuver """

        #self.stop_sign_start = rospy.get_rostime()
        self.crossing_sign = sign_frame_id
        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """
        within = self.within_range_of_stop(self.crossing_sign)
        if within == -1: return False
        return (self.mode == Mode.CROSS and not within)

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        #################################################################################
        # Do not change this for hw2 -- this won't affect your FSM since you are using gazebo
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        #################################################################################

    # YOUR STATE MACHINE
    # Currently it will just go to the pose without stopping at the stop sign

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        if len(self.delivery_targets) > 0 and self.close_to(self.x_g,self.y_g,self.theta):
            self.get_next_target()
            if len(self.delivery_targets) == 0: #shopping done, come home
                self.delivery_targets.append('return_to_home')
            elif self.delivery_targets[0] in 'return_to_home':
                self.delivery_targets = []


        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            for frame in self.trans_listener.getFrameStrings():
                #Can add more if cases here for different results (for example, navigate around puddle)
                if "stop_sign" in frame: #ie stop_sign_1, stop_sign_2, etc
                    self.detect_stop(frame)
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        elif self.mode == Mode.STOP:
            while not self.has_stopped():
                self.stay_idle()
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.init_crossing()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            while not self.has_crossed():
                self.nav_to_pose()
                if self.close_to(self.x_g,self.y_g,self.theta_g):
                    self.mode = Mode.IDLE
                    break
                    
            if not self.mode==Mode.IDLE:
                self.mode = Mode.NAV 

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                for frame in self.trans_listener.getFrameStrings():
                    #Can add more if cases here for different results (for example, navigate around puddle)
                    if "stop_sign" in frame: #ie stop_sign_1, stop_sign_2, etc
                        self.detect_stop(frame)
                self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    ############ your code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
