#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
import map_functions

import tf


class Lab3:

    def __init__(self):
        rospy.init_node('lab3', anonymous=True)
        rospy.wait_for_service('plan_path')
        self.plan_path = rospy.ServiceProxy('plan_path', GetPlan)
        self.pose = PoseStamped()
        self.pub_goal = rospy.Publisher('curr_goal', Path)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.find_and_follow_path)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.listener = tf.TransformListener()
        rospy.sleep(2)
        rospy.loginfo("lab3 node ready")

    def find_and_follow_path(self, msg):
        my_pose = PoseStamped()
        my_pose.pose = self.pose
        path_response = self.plan_path(my_pose, msg, 0.1)
        rospy.loginfo("path: "+str(path_response.plan.poses))
        self.pub_goal.publish(path_response.plan)


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        new_pose = PoseStamped()
        new_pose.pose = msg.pose.pose
        new_pose.header.frame_id = "odom"
        self.pose = self.listener.transformPose("map", new_pose)
        self.pose = self.pose.pose

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    l3 = Lab3()
    l3.run()