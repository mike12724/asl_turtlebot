#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Header
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Point, PointStamped, TransformStamped, Vector3
from asl_turtlebot.msg import DetectedObject
import math
import tf
from enum import Enum
from collections import defaultdict 
from copy import deepcopy

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

DIST_THRESHOLD = 0.5

class obj_loc_broadcaster:
    def __init__(self):
        rospy.init_node('object_position_broadcaster', anonymous=True)
        rospy.Subscriber('/supervisor/object_position', PoseStamped, self.add_new_tf)
        self.trans =  tf.TransformListener()        
        self.br = tf.TransformBroadcaster()
        self.map = defaultdict(int) 
        self.origin_frame = "/map" if mapping else "/odom"

        self.br_data = [] #list of all transforms to broadcast

    def add_new_tf(self,msg):
        head = msg.header
        frame = head.frame_id
        frame = frame.split('/')
        while len(frame[0]) == 0:
            del frame[0]

        base_frame = '/'+frame[0]
        obj_type = frame[1]
        obj_frame_id = '/'+obj_type+'_'+str(self.map[obj_type])

        msg.header.frame_id = base_frame

        new_ps = self.trans.transformPose(self.origin_frame, msg)

        in_map = False
        posit = new_ps.pose.position
        ori = new_ps.pose.orientation


        for i in range(self.map[obj_type]):
            #keep broadcasting to prevent loss of new (temp) frame
            while True:
                try:
                    new_ps.header.stamp = self.trans.getLatestCommonTime(self.origin_frame, self.origin_frame)
                    ps = self.trans.transformPose('/'+obj_type+'_'+str(i), new_ps)
                    break

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.loginfo(e)
            
            trans = ps.pose.position

            dist = math.sqrt(trans.x**2 + trans.y**2)
            if dist < DIST_THRESHOLD: #already in map
                in_map = True
                break

        #UPDATE MAP COUNT
        if not in_map:
            # transform = TransformStamped()
            # transform.header.stamp = rospy.Time(0) #hopefully this is okay
            # transform.header.frame_id = self.origin_frame #parent is the map
            # transform.child_frame_id = obj_frame_id #child is us
            # transform.transform.rotation = ori
            # transform.transform.translation = Vector3(posit.x, posit.y, posit.z)
            self.br_data.append([(posit.x,posit.y,posit.z),(ori.x, ori.y, ori.z, ori.w), obj_frame_id, self.origin_frame])
            #self.trans.setTransform(transform) #make it permanent
            self.map[obj_type] += 1
            rospy.loginfo(posit)

    def run(self):
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for item in self.br_data:
                self.br.sendTransform(item[0], item[1], rospy.Time().now(), item[2], item[3])
            self.rate.sleep()

if __name__ == '__main__':
    ob = obj_loc_broadcaster()
    ob.run()
