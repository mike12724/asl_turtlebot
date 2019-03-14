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

        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        self.pub_items = []

        self.trans_listener = tf.TransformListener()
        rospy.Subscriber('/cmd_nav', Pose2D, self.nav_callback)

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

    def nav_callback(self,msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta


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

    def create_object(self,frame_id,obj_type,obj_id,pos,ori,scale, color):
        robo_sphere = Marker()
        robo_sphere.header.frame_id = frame_id
        robo_sphere.header.stamp = rospy.Time()
        robo_sphere.ns = "robot"
        robo_sphere.id = obj_id
        robo_sphere.type = obj_type #sphere
        robo_sphere.action = 0 #add

        robo_sphere.pose.position.x = pos[0] #because we're in footprint id?
        robo_sphere.pose.position.y = pos[1]
        robo_sphere.pose.position.z = pos[2]

        robo_sphere.pose.orientation.x = ori[0] #because we're in footprint id?
        robo_sphere.pose.orientation.y = ori[1]
        robo_sphere.pose.orientation.z = ori[2]
        robo_sphere.pose.orientation.w = ori[3]

        robo_sphere.scale.x = scale[0]
        robo_sphere.scale.y = scale[1]
        robo_sphere.scale.z = scale[2]

        robo_sphere.color.a = 1.0
        robo_sphere.color.r = color[0]
        robo_sphere.color.g = color[1]
        robo_sphere.color.b = color[2]

        robo_sphere.lifetime = rospy.Duration(0)
        #robo_sphere.frame_locked = 1

        if obj_id == 2:
            for item in self.pub_items:
                if item.id == 2:
                    item.action=2
                    self.pub.publish(item)
                    self.pub_items.remove(item)


        if robo_sphere not in self.pub_items:
            self.pub_items.append(robo_sphere)



    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        origin_frame = "/map" if mapping else "/odom"
        self.create_object("/base_footprint",2,0,[0.,0.,0.],[0.,0.,0.,1.],[.15,.15,.15],[0,0,1.])

        #publish arrow marker on bot, in direction of pose (blue)
        self.create_object("/base_footprint",0,1,[0,0,0],[0,0,0,0],[.15,.03,.1],[0,0,1])
        #publish arrow marker on bot, in direction of velocity (green)

        while not rospy.is_shutdown():    
            z = tf.transformations.quaternion_from_euler(0,0,self.theta_g)
            self.create_object(origin_frame,0,2,[self.x_g,self.y_g,0],[z[0],z[1],z[2],z[3]],[.15,.06,.1],[0,0,1]) 
            for item in self.pub_items:
                self.pub.publish(item) 
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    if rviz:
        vis = Visualizer()
        vis.run()
    else:
        rospy.loginfo("Not using rviz")