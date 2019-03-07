#!/usr/bin/env python

"""
Run this test as we ran the hw2 turtlebot sim with stop signs.
1, make sure this node is compiled
  (I copied to the asl_turtlebot version in my catkin workspace and then remade the catkin workspace)
2, change the turtlebot3_signs_sim.launch file to use tensorflow: "use_tf" should be true.
3, run the launch file:
    roslaunch asl_turtlebot turtlebot3_signs_sim.launch
4, run this node:
    rosrun asl_turtlebot supervisor.py
5, publish the goal:
    rostopic pub /nav_pose geometry_msgs/Pose2D -- "4.0" "0.0" "0.0"
6, echo the stop sign topic:
    rostopic echo /detector/stop_sign
You should see output like the below when approaching the stop sign:

    ---
    id: 2
    name: "stop_sign"
    confidence: 99
    distance: 1.47614938021
    thetaleft: 0.254595916957
    thetaright: 0.19724214864
    corners: [0.0, 0.0, 0.0, 0.0]
    ---

TODO: Integrate LIDAR data into our extract_location function!

NOTE GM: I acutally haven't used feature_mapper.py at all, but this works.

MK: Edited the world file to have a stop sign at (2,1) and (5,-1). Do the following command to test:
    rostopic pub /nav_pose geometry_msgs/Pose2D -- "7.0" "0.0" "0.0"

"""

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Header
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Point, PointStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import detector # for loading object labels
from collections import defaultdict 

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

#threshold for considering two detected objects as the same
DETECT_EPS = 0.3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = 1.3

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
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.stop_sign_loc = None #for checking if we've successfully crossed
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # publish to obj_loc_broadcaster
        self.obj_loc_publisher = rospy.Publisher('/supervisor/object_position', PoseStamped, queue_size=10)


        # subscribers
        # stop sign detector
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
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
            if class_label != "stop_sign": continue
            print("INFO: creating subscriber for: {}".format(class_label))
            rospy.Subscriber(
                '/detector/'+class_label, 
                DetectedObject, 
                self.extract_object_location)
    
    # NOTE GM -- CURRENTLY NOT USING THIS
    # def image_callback(self,msg):
    #     #detector and detector_mobilenet use tf graphs
    #     # see scripts/detector.py:230 for details of the message, it is a DetectedObject
    #     obj_bounding_box = msg.corners
    #     #[ymin, xmin, ymax, xmax] = obj_bounding_box
    #     obj_id = msg.id
    #     obj_label = msg.name # the human readable class name
    #     obj_score = msg.confidence # how confident are we in the classification
    #     obj_dist = msg.distance
    #     obj_theta_left = msg.thetaleft
    #     obj_theta_right = msg.thetaright
    #     self.detected_object = msg
    #     self.extract_object_location(msg)
    #     # TODO: is this really all we need?

    #     #And subscribe to them, then get the bounding box, and object label
    #     #self.detected_object = (bb, label)
    #     #Could probably run extract_object_location from here

    def vel_callback(self,msg):
        self.V = msg.linear.x
    
    def extract_object_location(self,msg):
        # pass TODO
        #use self.detected_object, self.lidar_list, and tf transforms to 
        #figure out where self.detected_object is in terms of the map frame
        #self.map.append((x_location, y_location, object label)) 
        #may need an additional transform b/w base_footprint and camera (if camera not on center of robot)
            #need to see what tfs available in rviz when running gmapping_sim.launch for example

        #Ideally, we can also skip over duplicates - if we've put something into self.map already, no need to process it again

        # [ymin, xmin, ymax, xmax] = msg.corners
        # yc, xc         = 0.5*(ymin + ymax), 0.5*(xmin + xmax)
        th_r, th_l     = msg.thetaright, msg.thetaleft

        # Switch from [0,2pi] wrap to [-pi, pi]
        if th_r > math.pi:
            th_r = th_r - 2*math.pi
        if th_l > math.pi:
            th_l = th_l - 2*math.pi
        th_c = 0.5*(th_r + th_l) # THIS IS AN APPROX

        dist = msg.distance #FIXME WITH LIDAR
        x_from_cam = dist * math.cos(th_c)
        y_from_cam = dist * math.sin(th_c)
        theta_from_cam = math.pi+self.theta #assume sign is directly facing us

        if dist > 0.0:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time(0)

            #detected relative to robot, Also need msg.name to figure out what type it is
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

        # x_bot, y_bot, th_bot = self.x,self.y,self.theta

        # x_loc = (x_from_cam * math.cos(th_bot) - y_from_cam * math.sin(th_bot)) + x_bot
        # y_loc = (x_from_cam * math.sin(th_bot) + y_from_cam * math.cos(th_bot)) + y_bot

        # r_loc = math.sqrt(x_loc**2 + y_loc**2)



        # if (dist > 0.0 and not self.in_map(msg.name, x_loc,y_loc)):
        #     self.map[msg.name].append((x_loc,y_loc, r_loc, th_bot+th_c)) #r, alpha of sign in world coords

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

    def detect_stop(self, sign_frame_id):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign

        try:
            t = self.trans_listener.getLatestCommonTime(sign_frame_id, '/base_footprint')
            (dist_1,rot) = self.trans_listener.lookupTransform(sign_frame_id, '/base_footprint', t)
            t.secs -= 1 #HYPER COARSE! GOING BACK A FULL SECOND!
            (dist_2,rot) = self.trans_listener.lookupTransform(sign_frame_id, '/base_footprint', t)

            rospy.loginfo((dist_1[0], dist_2[0], dist_1[0]- dist_2[0]))
            if dist_2[0] > 0 and dist_2[0] < STOP_MIN_DIST and abs(dist_2[1]) < STOP_MIN_DIST and dist_2[0] - dist_1[0] > 0.1 and self.mode == Mode.NAV: #if in front of sign, and closer than 1 second ago (by some error margin)
                self.init_stop_sign()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        # stop_x = data[0]
        # stop_y = data[1]
        # stop_r = data[2]
        # stop_alph = data[3]

        # correct_side = (stop_r - self.x*math.cos(stop_alph) - self.y*math.sin(stop_alph)) > 0
        # correct_direction = (2*(self.V>0)-1)*self.theta
        # while correct_direction > math.pi:
        #     correct_direction -= 2*math.pi
        # while correct_direction < -math.pi:
        #     correct_direction += 2*math.pi
        # while stop_alph > math.pi:
        #     stop_alph -= 2*math.pi
        # while stop_alph < -math.pi:
        #     stop_alph += 2*math.pi
        # correct_direction = (abs(stop_alph - correct_direction) < math.pi/3) #traveling within 120 degrees of the stop sign

        # dist = math.sqrt((self.x - stop_x)**2 \
        #             + (self.y - stop_y)**2)

        # # if close enough and in nav mode, stop
        # if dist > 0 and dist < STOP_MIN_DIST and correct_direction and correct_side and self.mode == Mode.NAV:
        #     self.stop_sign_loc = data
        #     self.init_stop_sign()


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

        self.pose_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

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
        stop_x = self.stop_sign_loc[0]
        stop_y = self.stop_sign_loc[1]
        stop_r = self.stop_sign_loc[2]
        stop_alph = self.stop_sign_loc[3]
        dist = math.sqrt((self.x - stop_x)**2 + (self.y - stop_y)**2)
        correct_side = (stop_r - self.x*math.cos(stop_alph) - self.y*math.sin(stop_alph)) > 0
        return (self.mode == Mode.CROSS and (dist > STOP_MIN_DIST or not correct_side))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        TESTING = False

        if TESTING:
            target = "stop_sign"
            if target in self.map:
                # TESTING
                estimator_msg = DetectedObject()
                estimator_msg.id = 22
                estimator_msg.name = "stop_sign"
                estimator_msg.confidence = 99
                estimator_msg.distance = math.sqrt((self.x - self.map[target][0][0])**2 \
                    + (self.y - self.map[target][0][1])**2)
                estimator_msg.thetaleft = self.map[target][0][0]
                estimator_msg.thetaright = self.map[target][0][1]
                estimator_msg.corners = [0, 0, 0, 0]#[dist, x_from_cam, y_from_cam, th_c]
                self.location_publisher.publish(estimator_msg)

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

        #rospy.loginfo(self.map)
        for frame in self.trans_listener.getFrameStrings():
            #Can add more if cases here for different results (for example, navigate around puddle)
            if "stop_sign" in frame: #ie stop_sign_1, stop_sign_2, etc
                self.detect_stop(frame)

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            while not self.has_stopped():
                self.stay_idle()
            self.init_crossing()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            while not self.has_crossed():
                self.nav_to_pose()
            self.mode = Mode.NAV

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
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
