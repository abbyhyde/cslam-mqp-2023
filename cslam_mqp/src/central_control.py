#!/usr/bin/env python3

import map_functions, robot_behavior
import numpy as np
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf

class robot:
    def __init__(self, name):
        self.name = name
        self.tf_status = False #does the transfrom from world to name/map exist
        self.pose  = None #pose in grid coords

class swarm_behavior:
    #should getting a centroid be a service robots call?
    #when doing things with data that hs any possibility of changing over the runtime of a function make a deep copy!!!!
    #stale info should* cause less problems than a mix of stale and new data.
    #*i say should bc new data should be used in the next round

    def __init__(self):
        """
        Class constructor
        """
        rospy.sleep(20)
        rospy.init_node('swarm_behavior', anonymous=True)
        self.merged_map = None
        self.map_sub = rospy.Subscriber("world",OccupancyGrid, self.update_map)
        self.listener = tf.TransformListener()
        self.robots = [robot("LUIGI"), robot("WARIO")]
        self.centroid_pubs = []
        self.odom_subs= []
        for r in self.robots:
            self.centroid_pubs.append(rospy.Publisher('/' + r.name + '/centroid', PoseStamped))
            self.odom_subs.append(rospy.Subscriber('/' + r.name + '/odom', Odometry, self.update_odometry))
        rospy.loginfo("master_behavior node ready")
        
    
    def update_map(self, msg):
        self.merged_map = msg

    def update_odometry(self, msg):
        robot_index = 0
        for i in range(len(self.robots)):
            if msg._connection_header["topic"].split('/')[1] == self.robots[i].name:
                robot_index = i
                break
        new_pose = PoseStamped()
        new_pose.pose = msg.pose.pose
        new_pose.header.frame_id =  msg._connection_header["topic"].split('/')[1] + "/map"
        new_pose = self.listener.transformPose("world", new_pose).pose
        if self.merged_map is not None and self.robots[robot_index].tf_status:
            self.robots[robot_index].pose = map_functions.world_to_grid(self.merged_map, new_pose.position)
    
    def get_ready_robots(self):
        #returns the ready robots' indicies
        ready = []
        for i in range(len(self.robots)):
            r = self.robots[i]
            if r.tf_status and r.pose is not None:
                ready.append(i)
        return ready

    def run(self):
        '''TODO
        once the merged map exists, 
            calculate frontiers on the merged map
            distribute frontiers to robots if the tf from world to name/map exists and the robot is not currently busy
                do the distribution based on current distance (as the crow flies)
            repeat untill there are no more frontiers to map (propbably should wait for a bit to do it again)
            robot returns home if it doesn't get a frontier and isn't busy
        once no frontiers exist robot will return home
        '''
        while self.merged_map == None:
            rospy.sleep(3)
        last_frontiers = 1
        listener = tf.TransformListener()
        while last_frontiers > 0:
            
            for i in range(len(self.robots)): #test to make sure what robots have a transform with world
                try:
                    (trans,rot) = listener.lookupTransform('/world', '/' + self.robots[i].name + '/map', rospy.Time(0))
                except Exception as e:
                    self.robots[i].tf_status = False
                    continue
                print(self.robots[i].name + " has a tf")
                self.robots[i].tf_status = True

            map_data = self.merged_map
            ready_robots = self.get_ready_robots()
            rospy.loginfo(str(len(ready_robots)) + " robots are ready")
            if len(ready_robots) > 0: #dont process the map if there are no robots to receive a new target
                map_data = map_functions.dilate_map(map_data, math.ceil(0.110/map_data.info.resolution))  # expand the Cspace
                frontier = robot_behavior.threshold_frontiers(map_data[0])  # find the frontiers
                blobs = robot_behavior.find_blobs(frontier)
                centroids = robot_behavior.find_centroids(blobs)
                rospy.loginfo("we found " + str(len(centroids)) + " centroids")
                
                last_frontiers = len(centroids)
                for i in range(len(ready_robots) if len(ready_robots) <= len(centroids) else len(centroids)):
                    centroid_positions = np.array(centroids)[:,0:2]
                    weights = np.array(centroids)[:,2] 
                    for j in range(weights.shape[0]):
                        weights[j] *= 9/(map_functions.euclidean_distance(self.robots[ready_robots[i]].pose[0],self.robots[ready_robots[i]].pose[1], centroid_positions[j][0], centroid_positions[j][1])**2)
                        if(len(ready_robots) > 1):
                            weights[j] *= (2*math.atan(map_functions.euclidean_distance(self.robots[not int(ready_robots[i])].pose[0],self.robots[not int(ready_robots[i])].pose[1], centroid_positions[j][0], centroid_positions[j][1])**2)/math.pi)**2
                    chosen_index = np.argsort(weights)[0]
                    centroid_pose = PoseStamped()
                    centroid_pose.header.frame_id = 'world'
                    centroid_pose.pose.position = map_functions.grid_to_world(frontier, centroids[chosen_index][0], centroids[chosen_index][1])
                    self.centroid_pubs[ready_robots[i]].publish(centroid_pose)
                    centroids = np.delete(centroids, chosen_index, 0)

            
            rospy.sleep(2) # wait for robots to start/continue mapping
        
        #add going home when done


if __name__ == '__main__':
    swarm = swarm_behavior()
    swarm.run()