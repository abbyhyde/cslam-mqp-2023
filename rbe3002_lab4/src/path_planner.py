#!/usr/bin/env python3

import math
from priority_queue import PriorityQueue
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import Odometry, GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
import map_functions
import tf


class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.sleep(10)
        map_functions.init_request_map()
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        plan_path = rospy.Service('plan_path', GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.c_pub = rospy.Publisher('path_planner/cspace', GridCells, queue_size=10)
        self.c_pub2 = rospy.Publisher('path_planner/cspace2', OccupancyGrid, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.a_pub = rospy.Publisher('path_planner/astar', GridCells)
        self.pose = PoseStamped()
        self.graph = {}
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.update_odometry)
        self.listener = tf.TransformListener()
        ## Initialize the request counter
        self.request_counter = 0

        ## Sleep to allow roscore to do some housekeeping
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        poses = []
        old_position = None
        for (x, y) in path:
            pose_msg = PoseStamped()
            world_point = map_functions.grid_to_world(mapdata, x, y)
            pose_msg.pose.position = world_point
            # initial heading is set as zero
            quat = quaternion_from_euler(0, 0, 0)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            old_position = world_point
            poses.append(pose_msg)
        return poses

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        new_map, gridcells_msg = map_functions.dilate_map(mapdata, padding)
        rospy.loginfo("CSpace done")
        self.c_pub.publish(gridcells_msg)
        self.c_pub2.publish(new_map)
        return new_map

    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueue()
        frontier.put(start, 0)
        graph =  PathPlanner.make_graph(mapdata)
        gridcells_msg = GridCells()  # for c space publish
        gridcells_msg.cell_width = mapdata.info.resolution
        gridcells_msg.cell_height = mapdata.info.resolution
        gridcells_msg.cells = []
        gridcells_msg.header.frame_id = rospy.get_namespace() + "map"
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()
            gridcells_msg.cells = []
            # rospy.sleep(0.001)
            if current == goal:
                break

            for new_node in graph[current]:
                new_cost = cost_so_far[current] + (1 if new_node[0] == current[0] or new_node[1] == current[1] else 1.4)
                if new_node not in cost_so_far or new_cost < cost_so_far[new_node]:
                    cost_so_far[new_node] = new_cost
                    wall_bonus = 0
                    # if len(graph[new_node]) != 8:
                    #     wall_bonus = (8 - len(graph[new_node]))*3
                    priority = (new_cost + map_functions.euclidean_distance(goal[0], goal[1], new_node[0], new_node[1]) + wall_bonus)
                    frontier.put(new_node, priority)
                    came_from[new_node] = current
        path = []
        current = goal
        while current is not None:  # travel backwards to reach the start
            path.append(current)
            current = came_from[current]
        return path[::-1]  # return the reverse of that array

    @staticmethod
    def make_graph(mapdata):
        """
        :param mapdata: [OccupancyGrid] The map data.
        :return: {} library of index nodes
        """
        graph = {}
        for x in range(mapdata.info.width):
            for y in range(mapdata.info.height):  # iterate through all cells
                if map_functions.is_cell_walkable(mapdata, x, y):  # if we find an walkable cell
                    neighbors = set()
                    for point in map_functions.neighbors_of_8(mapdata, x, y):
                        neighbors.add(point)
                    graph[(x, y)] = neighbors
        rospy.loginfo("graph: " + str(graph))
        return graph

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        if len(path) > 1:
            path.pop(0)  # removing the first pose because it is not necessary
        x = 0
        while len(path) - 2 > x:
            if (path[x][0] == path[x + 2][0] or path[x][1] == path[x + 2][1]):  # if x or y of first point = x or y of third point
                del path[x + 1]
            else:
                x = x + 1

        rospy.loginfo("Optimizing path")
        return path

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        path_msg = Path()
        path_msg.poses = self.path_to_poses(mapdata, path)
        path_msg.header.frame_id = rospy.get_namespace() + "map"
        return path_msg

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        rospy.logdebug("start " + str(msg.start.pose.position))
        rospy.logdebug("end " + str(msg.goal.pose.position))
        self.request_counter = 1 + self.request_counter
        ## Request the map
        ## In case of error, return an empty path
        mapdata = map_functions.request_map()
        if mapdata is None:
            return []
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, math.ceil(0.110/mapdata.info.resolution)+1)
        ## Execute A*
        goal = map_functions.world_to_grid(mapdata, msg.goal.pose.position)
        offsets = [(0, 0), (3, 0), (0, 3), (3, 3), (-3, 0), (0, -3),
                   (-3, -3), (3, -3), (-3, 3)]
        path = []
        for offset in offsets:
            start = map_functions.world_to_grid(mapdata, msg.start.pose.position)
            start_with_offset = (start[0] + offset[0], start[1] + offset[1])
            try:
                path = self.a_star(cspacedata, start_with_offset, goal)
                rospy.loginfo("found valid path")
                break
            except KeyError:
                rospy.loginfo("failed to find valid path")
                path = []
                        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)

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

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


if __name__ == '__main__':
    PathPlanner().run()