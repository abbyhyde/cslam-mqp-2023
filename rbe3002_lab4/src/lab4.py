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


class Lab4:

    def __init__(self):
        """
        Class constructor
        """
        rospy.sleep(10)
        rospy.init_node('lab4', anonymous=True)
        self.pub_goal = rospy.Publisher('frontier_goal', PoseStamped)
        self.f_pub = rospy.Publisher('frontier', GridCells)
        self.pose = PoseStamped()
        self.initial_pose = None
        self.listener = tf.TransformListener()
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.update_odometry)
        rospy.wait_for_service('plan_path')
        self.plan_path = rospy.ServiceProxy('plan_path', GetPlan)
        self.pub_goal = rospy.Publisher('curr_goal', Path)
        map_functions.init_request_map()
        self.bad_frontier = False
        self.part_one_done = False
        rospy.sleep(25)
        rospy.loginfo("lab4 node ready")

    def threshold_frontiers(self, mapdata):
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
        self.f_pub.publish(gridcells_msg)
        return new_map

    @staticmethod
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

    @staticmethod
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

    @staticmethod
    def rank_centroid(centroids, robot_position):
        """
        currently choses the larget centroid
        :param bad_frontier:  [bool] choose the second-best frontier
        :param centroids: [[(int, int, int)]] array centroid positions with areas
        :return: [int, int] a grid cell to navigate to
        """
        rospy.loginfo("ranking centroids")
        rank = PriorityQueue()
        for centroid in centroids:
            h = centroid[2] / (0.0001 + map_functions.euclidean_distance(centroid[0], centroid[1],
                                                                robot_position[0], robot_position[1]) ** 3)
            rank.put(centroid, 1 / h)
        return rank

    def part_one(self):
        """
        behavior for part one of the lab
        """
        last_try_failed = False
        last_pose = PoseStamped()
        last_pose.pose.position = self.pose.position
        last_pose.header.frame_id = rospy.get_namespace() + "odom"
        while True:
            map_data = map_functions.request_map()
            map_data = map_functions.dilate_map(map_data, math.ceil(0.110/map_data.info.resolution))  # expand the Cspace
            frontier = self.threshold_frontiers(map_data[0])  # find the frontiers
            blobs = Lab4.find_blobs(frontier)
            centroids = Lab4.find_centroids(blobs)
            robot_position = map_functions.world_to_grid(frontier, self.pose.position)
            ranked_centroids = Lab4.rank_centroid(centroids, robot_position)
            path_response = None
            empty_path = Path()
            empty_path.poses = []
            empty_path.header.frame_id = rospy.get_namespace() + "map"
            self.pub_goal.publish(empty_path)
            try:
                while path_response is None or len(path_response.plan.poses) == 0:
                    target = ranked_centroids.get()
                    goal_pose = PoseStamped()
                    goal_pose.pose.position = map_functions.grid_to_world(frontier, target[0], target[1])
                    goal_pose.header.frame_id = rospy.get_namespace() + "map"
                    path_response = self.plan_path(last_pose, goal_pose, 0.1)
                    if len(path_response.plan.poses) != 0:
                        last_try_failed = False
                        rospy.loginfo("found good frontier")
                        rospy.loginfo("path: " + str(path_response.plan.poses))
                        self.pub_goal.publish(path_response.plan)
                        last_pose = goal_pose
                        last_pose.header.frame_id = rospy.get_namespace() + "map"
            except IndexError:
                if last_try_failed:
                    break
                else:
                    last_try_failed = True
            if last_try_failed:
                rospy.sleep(15)
            else:
                rospy.sleep(12)  # wait for the map to update #TODO dynamicly set this by path length
        rospy.loginfo("Done Mapping")
        goal_pose = PoseStamped()
        goal_pose.pose = self.initial_pose
        goal_pose.header.frame_id = rospy.get_namespace() + "map"
        my_pose = PoseStamped()
        my_pose.pose = self.pose
        path_response = self.plan_path(my_pose, goal_pose, 0.1)
        rospy.loginfo("path: " + str(path_response.plan.poses))
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
        self.pose = self.listener.transformPose(rospy.get_namespace() + "map", new_pose)
        self.pose = self.pose.pose
        if self.initial_pose is None:
            self.initial_pose = self.pose

    def update_bad_frontier(self, message):
        self.bad_frontier = message

    def run(self):
        self.part_one()
        rospy.spin()


if __name__ == '__main__':
    l4 = Lab4()
    l4.run()