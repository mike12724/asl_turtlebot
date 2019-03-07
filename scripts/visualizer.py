#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "rviz = %s\n" % rviz
print "mapping = %s\n" % mapping


class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.V = 0
        self.om = 0

        self.trans_listener = tf.TransformListener()
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)

        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        
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

    def velocity_callback(self,msg):
        self.V = msg.linear.x
        self.om = msg.angular.z


    def loop(self):
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



    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        #publish sphere marker on bot
        robo_sphere = Marker()
        robo_sphere.header.frame_id = "/base_footprint"
        robo_sphere.header.stamp = rospy.Time()
        robo_sphere.ns = "robot"
        robo_sphere.id = 0
        robo_sphere.type = 2 #sphere
        robo_sphere.action = 0 #add

        robo_sphere.pose.position.x = 0 #because we're in footprint id?
        robo_sphere.pose.position.y = 0
        robo_sphere.pose.position.z = 0

        robo_sphere.pose.orientation.x = 0 #because we're in footprint id?
        robo_sphere.pose.orientation.y = 0
        robo_sphere.pose.orientation.z = 0
        robo_sphere.pose.orientation.w = 0

        robo_sphere.scale.x = 0.15
        robo_sphere.scale.y = 0.15
        robo_sphere.scale.z = 0.15

        robo_sphere.color.a = 1.0
        robo_sphere.color.r = 0.0
        robo_sphere.color.g = 0.0
        robo_sphere.color.b = 1.0

        robo_sphere.lifetime = rospy.Duration(0)
        robo_sphere.frame_locked = 1

        #publish arrow marker on bot, in direction of pose (blue)

        #publish arrow marker on bot, in direction of velocity (green)

        while not rospy.is_shutdown():     
            self.pub.publish(robo_sphere) 
            #self.loop()
            rate.sleep()


if __name__ == '__main__':
    if rviz:
        vis = Visualizer()
        vis.run()
    else:
        rospy.loginfo("Not using rviz")