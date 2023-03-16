#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class amcl_transform:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('amcl_transform_creator', anonymous=True)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.sub_goal = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.set_transform)
        self.px = 0
        self.py = 0
        self.pth = 0
        self.poses = []
        rospy.sleep(10)
        rospy.loginfo("amcl_transform node ready")


    def set_transform(self, msg):
        """
        creats the transform from the world to this robot's
        :param msg [PoseWithCovarianceStamped] pose from amcl
        """

        
        robot_frame_id = rospy.get_namespace() + "map"
        robot_base_frame = rospy.get_namespace() + "base_footprint"
        pose = msg.pose.pose

        try:
            (trans,rot) = self.listener.lookupTransform(robot_frame_id, robot_base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)  as err:
            rospy.loginfo("amcl_transform node has a problem looking up the transform: " + str(err))
            return

        old_roll, old_pitch, old_yaw = euler_from_quaternion(rot)
        new_roll, new_pitch, new_yaw  = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        # - old_yaw + math.pi/2
        self.broadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z), quaternion_from_euler(new_roll, new_pitch, new_yaw), rospy.Time.now(), robot_frame_id, "/world")
        rospy.loginfo("amcl_transform node received a transform " + str((pose.position.x - trans[0],pose.position.y - trans[1], pose.position.z - trans[2])))
    
    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    create_transform = amcl_transform()
    create_transform.run()