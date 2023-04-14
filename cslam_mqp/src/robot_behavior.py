#!/usr/bin/env python3
import math

import rospy
import map_functions
from priority_queue import PriorityQueue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, GridCells, OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
import tf
import scipy
import numpy as np


MAX_ATTEMPTS = 3

def threshold_frontiers(mapdata):
    """
    creates a binary frontier map
    :param mapdata: [OccupancyGrid] The map data.
    :return: mapdata: [OccupancyGrid] The map data with the frontiers marked negative and everything else zero
    """
    rospy.loginfo("Creating Frontier Map")
    ## Go through each cell in the occupancy grid
    ## Inflate the obstacles where necessary
    new_map = OccupancyGrid()
    new_map.info = mapdata.info
    new_map.data = [0] * (new_map.info.width * new_map.info.height)
    gridcells_msg = GridCells()
    gridcells_msg.header.frame_id = rospy.get_namespace() + "map"
    gridcells_msg.cell_width = new_map.info.resolution
    gridcells_msg.cell_height = new_map.info.resolution
    for x in range(new_map.info.width):
        for y in range(new_map.info.height):  # iterate through all cells
            if map_functions.is_cell_unknown(mapdata, x, y):  # if cell is unknown
                for cell in map_functions.neighbors_of_8(mapdata, x, y):  # If cell is unknown
                    if abs(cell[0] - x) > 1 or abs(cell[1] - y) > 1:
                        rospy.loginfo("Something failed")
                    else:
                        new_map.data[map_functions.grid_to_index(new_map, cell[0], cell[1])] = 1  # Mark it as -1
                        gridcells_msg.cells.append(map_functions.grid_to_world(new_map, x, y))
    new_map.data = (scipy.ndimage.binary_erosion(scipy.ndimage.binary_dilation(np.array(new_map.data)\
        .reshape((new_map.info.width, new_map.info.height)), iterations=2), iterations=2).reshape(len(new_map.data))*-1).tolist()
    return new_map

def find_blobs(mapdata):
    """
    finds blobs of frontiers, returns a list of lists of tuples
    :param mapdata: [OccupancyGrid] The map data.
    :return: [[[(int, int)]]] frontier groups
    """
    rospy.loginfo("Finding Blobs")
    frontiers = []
    for x in range(mapdata.info.width):
        for y in range(mapdata.info.height):
            if map_functions.is_cell_unknown(mapdata, x, y):
                alreadyChecked = False
                for blob in frontiers:  # check to see if the unknown square is has been put into a blob yet
                    if (x, y) in blob:
                        alreadyChecked = True
                        break
                if not alreadyChecked:  # if not construct a new blob starting at this blob
                    newBlob = []
                    cellsToCheck = []
                    cellsToCheck.append((x, y))
                    while len(cellsToCheck) > 0:
                        currCell = cellsToCheck.pop(0)
                        newBlob.append(currCell)
                        for dx in range(-1, 2):
                            for dy in range(-1, 2):
                                if (map_functions.is_cell_unknown(mapdata, currCell[0] + dx, currCell[1] + dy) and
                                        (currCell[0] + dx, currCell[1] + dy) not in newBlob and
                                        (currCell[0] + dx, currCell[1] + dy) not in cellsToCheck):
                                    cellsToCheck.append((currCell[0] + dx, currCell[1] + dy))
                    frontiers.append(newBlob)
    return frontiers

def find_centroids(frontiers):
    """
    finds the centroids of the frontier on the thresholded map
    :param mapdata: [[[(int, int)]]] frontier groups
    :return: [[(int, int, int)]] array centroid positions with areas (x_pos, y_pos, area)
    """
    rospy.loginfo("Finding the centroids of the frontier")
    List_Centroids = []
    for blob in frontiers:
        length = len(blob)
        area = length
        average = np.zeros_like(blob)
        average[:,0] = np.mean(blob, axis = 0)[0]
        average[:,1] = np.mean(blob, axis = 0)[1]
        center_index = np.argmin(np.sum((np.array(blob)-average)**2, axis = 1)**0.5)
        print(center_index)
        position = (blob[center_index][0], blob[center_index][1], area)
        List_Centroids.append(position)
    return List_Centroids

class robot:

    def __init__(self):
        """
        Class constructor
        """
        self.mapping = False
        self.donecspace = False
        self.goal_pose = None
        rospy.sleep(10)
        rospy.init_node('robot_behavior', anonymous=True)
        self.sub_goal = rospy.Subscriber('centroid', Path, self.explore_centroid)
        self.pose = PoseStamped()
        self.initial_pose = None
        self.listener = tf.TransformListener()
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.update_odometry)
        rospy.wait_for_service('plan_path')
        self.plan_path = rospy.ServiceProxy('plan_path', GetPlan)
        self.path_home = rospy.Service('path_home', GetPlan, self.go_home)
        self.pub_goal = rospy.Publisher('curr_goal', Path, queue_size=1)
        map_functions.init_request_map()
        rospy.sleep(25)
        rospy.loginfo("robot_behavior node ready")

    def explore_centroid(self, msg):
        """
        behavior for exploration
        """
        if not self.mapping:
            self.mapping = True
            rospy.loginfo("received frontiers" + str(len(msg.poses)))
            map_data = map_functions.request_map()
            map_data = map_functions.dilate_map(map_data, math.ceil(0.110/map_data.info.resolution)+1)  # expand the Cspace
            frontier = threshold_frontiers(map_data[0])  # find the frontiers
            blobs = find_blobs(frontier)
            centroids = find_centroids(blobs)
            #compare centroid received from central control
            new_path = Path()
            new_path.header.frame_id = rospy.get_namespace() + "map"
            while len(msg.poses) > 0:
                last_pose = PoseStamped()
                if(len(new_path.poses) == 0):
                    last_pose.pose.position = self.pose.position
                else:
                    last_pose.pose.position = new_path.poses[-1].pose.position

                goal_pose = PoseStamped()
                goal_pose.pose = msg.poses[0].pose
                chosen_pose = msg.poses[0]
                for p in msg.poses:
                        if(map_functions.euclidean_distance(p.pose.position.x, p.pose.position.y, last_pose.pose.position.x, last_pose.pose.position.y) 
                        < map_functions.euclidean_distance(goal_pose.pose.position.x, goal_pose.pose.position.y, last_pose.pose.position.x, last_pose.pose.position.y)):
                            goal_pose.pose = p.pose
                            chosen_pose = p
                msg.poses.remove(chosen_pose)
                goal_pose.header.frame_id = "/world"
                goal_pose = self.listener.transformPose(rospy.get_namespace() + "map", goal_pose)
                print("transformed the pose")
                goal_position = map_functions.world_to_grid(frontier, goal_pose.pose.position)
                if len(centroids) > 0:
                    distances = []
                    for centroid in centroids:
                        distances.append(map_functions.euclidean_distance(centroid[0],centroid[1], goal_position[0], goal_position[1]))
                    order = np.argsort(distances).tolist()
                    print("found the centroid")

                    path_response = None
                    failed_attempts = 0
                    while path_response is None and MAX_ATTEMPTS > failed_attempts and len(order) > 0:
                        target = centroids[order.pop(0)]
                        goal_pose = PoseStamped()
                        goal_pose.pose.position = map_functions.grid_to_world(frontier, target[0], target[1])
                        goal_pose.header.frame_id = rospy.get_namespace() + "map"
                        last_pose = PoseStamped()
                        if(len(new_path.poses) == 0):
                            last_pose.pose.position = self.pose.position
                        else:
                            last_pose.pose.position = new_path.poses[-1].pose.position
                        last_pose.header.frame_id = rospy.get_namespace() + "map"
                        rospy.loginfo(last_pose)
                         # stop momement before planning
                        path_response = self.plan_path(last_pose, goal_pose, 0.1)
                        if len(path_response.plan.poses) != 0:
                            rospy.loginfo("found good path to frontier")
                            new_path.poses.extend(path_response.plan.poses)
                            centroids.remove(target)
                        else:
                            failed_attempts += 1
                else:
                    rospy.loginfo("failed to find good path to frontier")
                    break
            if len(new_path.poses) > 0:
                goal_pose.pose = self.initial_pose
                goal_pose.header.frame_id = rospy.get_namespace() + "map"
                my_pose = PoseStamped()
                my_pose.pose = new_path.poses[-1].pose
                path_response = self.plan_path(my_pose, goal_pose, 0.1)
                new_path.poses.extend(path_response.plan.poses)
                self.pub_goal.publish(new_path)
            rospy.sleep(3) # make sure the message gets there before we do it again.
            self.mapping = False
        

    def go_home(self, msg):
        goal_pose = PoseStamped()
        goal_pose.pose = self.initial_pose
        goal_pose.header.frame_id = rospy.get_namespace() + "map"
        rospy.loginfo("heading home")
        my_pose = PoseStamped()
        my_pose.pose = self.pose
        path_response = self.plan_path(my_pose, goal_pose, 0.1)
        return path_response.plan

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        new_pose = PoseStamped()
        new_pose.pose = msg.pose.pose
        new_pose.header.frame_id = rospy.get_namespace() + "odom"
        self.pose = self.listener.transformPose(rospy.get_namespace() + "map", new_pose).pose
        if self.initial_pose is None:
            self.initial_pose = self.pose

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    r = robot()
    r.run()