#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.srv import GetPlan
import math
import tf
#TODO status publisher, possibly avoiding objects based on scan

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2', anonymous=True)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.set_path
        self.listener = tf.TransformListener()
        self.plan_path_home = rospy.ServiceProxy('path_home', GetPlan)
        self.sub_goal = rospy.Subscriber('curr_goal', Path, self.set_path)
        self.total_path_pub = rospy.Publisher('total_path', Path, queue_size=10)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.setDistance)
        self.mapping_pub = rospy.Publisher('mapping', Bool, queue_size=10)
        self.total_path = Path()
        self.total_path.header.frame_id = rospy.get_namespace() + 'map'
        self.px = 0
        self.py = 0
        self.pth = 0
        self.poses = []
        self.distance_infront = 0
        self.obstacle_count = 0
        rospy.sleep(35)
        rospy.loginfo("lab2 node ready")

    def setDistance(self, msg):
        FOV = 60*math.pi/180
        distance = msg.ranges[0]
        min_distance = msg.range_min
        for i in range(1, int(FOV/(msg.angle_increment*2))):
            if(msg.ranges[i] > min_distance and msg.ranges[i]*math.cos(msg.angle_increment*i) < distance):
                distance = msg.ranges[i]*math.cos(msg.angle_increment*i)
            if(msg.ranges[-i] > min_distance and msg.ranges[-i]*math.cos(msg.angle_increment*i) < distance):
                distance = msg.ranges[-i]*math.cos(msg.angle_increment*i)
        
        if(distance > min_distance):
            self.distance_infront = distance

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        # Create twist message
        msg_cmd_vel = Twist()

        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        # Angular velocity
        msg_cmd_vel.angular.z = angular_speed
        # Send command
        self.pub_vel.publish(msg_cmd_vel)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        heading_proportional = 1
        heaing_integral = 0.01
        errorSum = 0
        initial_x = self.px
        initial_y = self.py
        initial_th = self.pth
        if distance == 0:
            return
        self.send_speed(linear_speed * abs(distance) / distance, 0)
        while abs(math.sqrt(pow(initial_x - self.px, 2) + pow(initial_y - self.py, 2)) - abs(distance)) > 0.01:
            rospy.sleep(0.05)
        self.send_speed(0, 0)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param aspeed:       [float] [rad/s] The angular speed.
        :param angle         [float] [rad]   The distance to cover.
        """
        initial_th = self.pth
        if angle == 0:
            return
        direction = abs(self.angleDifference(self.clampAngle(angle + initial_th), self.pth)) / self.angleDifference(self.clampAngle(angle + initial_th), self.pth)
        self.send_speed(0, aspeed * direction)

        while abs(self.angleDifference(self.clampAngle(angle + initial_th), self.pth)) > 0.08:
            print("target:", self.clampAngle(angle + initial_th), " current: ", self.pth)
            self.send_speed(0, aspeed * direction)
            rospy.sleep(0.01)
        self.send_speed(0, 0)

    def clampAngle(self, angle):
        """
        clamps the input between -pi and pi
        :param   angle: [float] [rad]
        :return: angle: [float] [rad]
        """
        angle = angle % (2 * math.pi)
        if abs(angle) == math.pi:
            return math.pi
        elif angle > math.pi:
            return angle - (2 * math.pi)
        elif angle < -1 * math.pi:
            return angle + (2 * math.pi)
        return angle

    def angleDifference(self, a1, a2):
        """
        calculates the shortest difference between two angles
        :param a1: [float] [rad] first angle
        :param a2: [float] [rad] second angle
        :return: [float] [rad] angle between
        """
        difference = a1 - a2
        if abs(difference) < math.pi: #
            return difference
        return difference - (2 * math.pi * abs(difference) / difference)

    def set_path(self, msg):
        """
        Sets the path from a*
        This method is a callback bound to a Subscriber.
        :param msg:
        """
        if(len(self.poses) <= 0):
            rospy.loginfo("new path")
            self.mapping_pub.publish(Bool(True))
            self.poses = msg.poses
            self.total_path.poses.extend(self.poses)
            self.total_path_pub.publish(self.total_path)


    def go_to(self):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        while True:
            if len(self.poses) > 0:

                self.total_path_pub.publish(self.total_path)
                pose = self.poses.pop(0)
                
                rospy.loginfo(pose)
                next_pose = PoseStamped()
                next_pose.pose = pose.pose
                next_pose.header.frame_id = rospy.get_namespace() + "map"
                next_pose_in_odom = self.listener.transformPose(rospy.get_namespace() + "odom", next_pose)
                pose.pose = next_pose_in_odom.pose
                x_Target = pose.pose.position.x  # Get the x and y position from the message
                y_Target = pose.pose.position.y
                quat_orig = pose.pose.orientation  # Get the quaternion from the message
                quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
                (roll, pitch, yawTarget) = euler_from_quaternion(quat_list)  # convert the quaternion to euler angles
                angle = math.atan2(y_Target - self.py, x_Target - self.px)
                # rospy.loginfo("prerotate")
                self.rotate(self.angleDifference(angle, self.pth), 0.5)
                # rospy.loginfo("linear move")
                P = 1.7
                start = None
                while abs(math.sqrt(pow(x_Target - self.px, 2) + pow(y_Target - self.py, 2))) > 0.06:

                    self.mapping_pub.publish(Bool(True))
                    next_pose_in_odom = self.listener.transformPose(rospy.get_namespace() + "odom", next_pose)
                    pose.pose = next_pose_in_odom.pose
                    x_Target = pose.pose.position.x  # Get the x and y position from the message
                    y_Target = pose.pose.position.y
                    angle = math.atan2(y_Target - self.py, x_Target - self.px)
                    th_error = self.angleDifference(angle, self.pth)
                    weight = 1
                    if(self.distance_infront - 0.22 < 0): # within 2 cspaces
                        weight = (100*self.distance_infront -13)/9 # at 0.22 weight = 1 at 0.13 weight  = 0``
                        rospy.loginfo_throttle(1, str(weight) +" is the weight")
                        if weight < 0.15: #if weight is low enough for long enough, give up
                            weight = 0 
                            if start == None:
                                start = rospy.get_rostime()
                            elif rospy.get_rostime().secs - start.secs > 10:
                                rospy.loginfo("encountered an obstacle")
                                rospy.sleep(3)
                                self.obstacle_count += 1
                                if self.obstacle_count >= 2: #stuck for 20 seconds
                                    self.drive(-0.05, 0.15)
                                    self.poses = self.plan_path_home(PoseStamped(), PoseStamped(), 0.1).plan.poses
                                    self.obstacle_count = 0
                                    break
                                start = None
                                break
                        else:
                            self.obstacle_count = 0
                            start = None
                    self.send_speed(0.15*weight, th_error * P)
                    rospy.sleep(0.01)
                self.send_speed(0, 0)
            else:
                self.send_speed(0, 0)
                self.mapping_pub.publish(Bool(False))
                rospy.sleep(0.05)



    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.px = msg.pose.pose.position.x  # Get the x and y position from the message
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation  # Get the quaterion from the message
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)  # convert the quaterion to euler angles
        self.pth = yaw

    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass  # delete this when you implement your code

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass  # delete this when you implement your code

    def run(self):
        self.go_to()


if __name__ == '__main__':
    l2 = Lab2()
    l2.run()