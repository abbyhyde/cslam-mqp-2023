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


MAX_ATTEMPTS = 1

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
                        new_map.data[map_functions.grid_to_index(new_map, cell[0], cell[1])] = -1  # Mark it as -1
                        gridcells_msg.cells.append(map_functions.grid_to_world(new_map, x, y))
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
        centroid = blob[int(length / 2)]
        position = (centroid[0], centroid[1], area)
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
        self.sub_goal = rospy.Subscriber('centroid', PoseStamped, self.explore_centroid)
        self.pose = PoseStamped()
        self.initial_pose = None
        self.listener = tf.TransformListener()
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.update_odometry)
        rospy.wait_for_service('plan_path')
        self.plan_path = rospy.ServiceProxy('plan_path', GetPlan)
        self.pub_goal = rospy.Publisher('curr_goal', Path)
        map_functions.init_request_map()
        rospy.sleep(25)
        rospy.loginfo("robot_behavior node ready")

    def explore_centroid(self, msg):
        """
        behavior for exploration
        """
        if not self.donecspace:
            self.goal_pose = msg.pose

        if not self.mapping:
            self.mapping = True

            map_data = map_functions.request_map()
            map_data = map_functions.dilate_map(map_data, math.ceil(0.110/map_data.info.resolution))  # expand the Cspace
            frontier = threshold_frontiers(map_data[0])  # find the frontiers
            blobs = find_blobs(frontier)
            centroids = find_centroids(blobs)
            self.donecspace = True
            #compare centroid received from
            goal_pose = PoseStamped()
            goal_pose.pose = self.goal_pose
            goal_pose.header.frame_id = "/world"
            goal_pose = self.listener.transformPose(rospy.get_namespace() + "map", goal_pose)
            self.donecspace = False
            goal_position = map_functions.world_to_grid(frontier, goal_pose.pose.position)
            if len(centroids) > 0:
                target = centroids[0] #set to chosen centroid
                for centroid in centroids:
                    if(map_functions.euclidean_distance(centroid[0],centroid[1], goal_position[0], goal_position[1]) 
                    < map_functions.euclidean_distance(target[0],target[1], goal_position[0], goal_position[1])):
                        target = centroid

                path_response = None
                empty_path = Path()
                empty_path.poses = []
                empty_path.header.frame_id = rospy.get_namespace() + "map"
                failed_attempts = 0
                while path_response is None or len(path_response.plan.poses) == 0 and MAX_ATTEMPTS > failed_attempts:
                    goal_pose = PoseStamped()
                    goal_pose.pose.position = map_functions.grid_to_world(frontier, target[0], target[1])
                    goal_pose.header.frame_id = rospy.get_namespace() + "map"
                    last_pose = PoseStamped()
                    last_pose.pose.position = self.pose.position
                    last_pose.header.frame_id = rospy.get_namespace() + "map"
                    rospy.loginfo(last_pose)
                    empty_path = Path()
                    empty_path.poses = []
                    empty_path.header.frame_id = rospy.get_namespace() + "map"
                    self.pub_goal.publish(empty_path) # stop momement before planning
                    path_response = self.plan_path(last_pose, goal_pose, 0.1)
                    if len(path_response.plan.poses) != 0:
                        rospy.loginfo("found good path to frontier")
                        self.pub_goal.publish(path_response.plan)
                        last_pose.header.frame_id = rospy.get_namespace() + "map"
                        rospy.loginfo("waiting "+ str(2*len(path_response.plan.poses)))
                        rospy.sleep(2*len(path_response.plan.poses)) #wait for the robot to traverse the path
                    else:
                        failed_attempts += 1
            self.mapping = False
        

    def go_home(self, msg):
        goal_pose = PoseStamped()
        goal_pose.pose = self.initial_pose
        goal_pose.header.frame_id = rospy.get_namespace() + "map"
        my_pose = PoseStamped()
        my_pose.pose = self.pose
        path_response = self.plan_path(my_pose, goal_pose, 0.1)
        rospy.loginfo("heading home")
        self.pub_goal.publish(path_response.plan)

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