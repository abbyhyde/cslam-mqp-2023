#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import GridCells, OccupancyGrid

get_map = None


def grid_to_index(mapdata, x, y):
    """
    Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
    :param mapdata: [OccupancyGrid] The map information.
    :param x [int] The cell X coordinate.
    :param y [int] The cell Y coordinate.
    :return  [int] The index.
    """

    index = y * mapdata.info.width + x
    return index


def euclidean_distance(x1, y1, x2, y2):
    """
    Calculates the Euclidean distance between two points.
    :param x1 [int or float] X coordinate of first point.
    :param y1 [int or float] Y coordinate of first point.
    :param x2 [int or float] X coordinate of second point.
    :param y2 [int or float] Y coordinate of second point.
    :return   [float]        The distance.
    """
    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


def grid_to_world(mapdata, x, y):
    """
    Transforms a cell coordinate in the occupancy grid into a world coordinate.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The cell X coordinate.
    :param y       [int]           The cell Y coordinate.
    :return        [Point]         The position in the world.
    """

    # wc_x and wc_y are the world coordinates
    # x and y are the grid coordinates
    # resolution is the map resolution
    # mapdata.info.origin.position.x and mapdata.info.origin.position.y are the position of the origin in the world

    wc_x = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
    wc_y = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
    wc_point = Point()
    wc_point.x = wc_x
    wc_point.y = wc_y
    wc_point.z = 0

    return wc_point  # return a coordinate


def world_to_grid(mapdata, wp):
    """
    Transforms a world coordinate into a cell coordinate in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param wp      [Point]         The world coordinate.
    :return        [(int,int)]     The cell position as a tuple.
    """
    wc_x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
    wc_y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
    return wc_x, wc_y


def is_cell_walkable(mapdata, x, y):
    """
    A cell is walkable if all of these conditions are true:
    1. It is within the boundaries of the grid;
    2. It is free (not unknown, not occupied by an obstacle)
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [boolean]       True if the cell is walkable, False otherwise
    """
    ### REQUIRED CREDIT
    if x < 0 or y < 0 or x >= mapdata.info.width or y >= mapdata.info.height:
        return False  # check to see if the point is outside of the bounds
    if mapdata.data[grid_to_index(mapdata, x, y)] > 5 or mapdata.data[
        grid_to_index(mapdata, x, y)] < 0:  # thresh hold of free cell
        return False  # check to see if we are certain that the cell is walkable
    return True


def is_cell_unknown(mapdata, x, y):
    """
    A cell is unknown if all of these conditions are true:
    1. It is within the boundaries of the grid;
    2. It is negatibe
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [boolean]       True if the cell is unknown, False otherwise
    """
    ### REQUIRED CREDIT
    if x < 0 or y < 0 or x >= mapdata.info.width or y >= mapdata.info.height:
        return False  # check to see if the point is outside of the bounds
    if mapdata.data[grid_to_index(mapdata, x, y)] >= 0:  # thresh hold of unknown cell
        return False  # check to see if we are certain that the cell is unknown
    return True


def is_cell_occupied(mapdata, x, y):
    """
    A cell is occupied if all of these conditions are true:
    1. It is within the boundaries of the grid;
    2. It is positive
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [boolean]       True if the cell is unknown, False otherwise
    """
    ### REQUIRED CREDIT
    if x < 0 or y < 0 or x >= mapdata.info.width or y >= mapdata.info.height:
        return False  # check to see if the point is outside of the bounds
    if mapdata.data[grid_to_index(mapdata, x, y)] <= 0:  # thresh hold of occupied cell
        return False  # check to see if we are certain that the cell is occupied
    return True


def neighbors_of_4(mapdata, x, y):
    """
    Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [[(int,int)]]   A list of walkable 4-neighbors.
    """
    walkableSquares = []

    for dx in range(-1, 2, 2):  # east and west squares
        if is_cell_walkable(mapdata, x + dx, y):
            walkableSquares.append((x + dx, y))
    for dy in range(-1, 2, 2):  # north and south squares
        if is_cell_walkable(mapdata, x, y + dy):
            walkableSquares.append((x, y + dy))
    return walkableSquares


def neighbors_of_8(mapdata, x, y):
    """
    Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [[(int,int)]]   A list of walkable 8-neighbors.
    """
    walkableSquares = []

    for dx in range(-1, 2):
        for dy in range(-1, 2):
            if is_cell_walkable(mapdata, x + dx, y + dy) and not (dx == 0 and dy == 0):
                walkableSquares.append((x + dx, y + dy))
    return walkableSquares


def init_request_map():
    rospy.loginfo("Waiting for GetMap")
    rospy.wait_for_service('dynamic_map')
    global get_map
    get_map = rospy.ServiceProxy('dynamic_map', GetMap)
    rospy.loginfo("Done waiting for GetMap")


def request_map():
    """
    Requests the map from the map server.
    :return [OccupancyGrid] The grid if the service call was successful,
                            None in case of error.
    """
    ### REQUIRED CREDI
    global get_map
    requestedMap = get_map()
    if requestedMap is not None:
        rospy.loginfo("map found, map width: %d, map height: %d", requestedMap.map.info.width,
                      requestedMap.map.info.height)
        return requestedMap.map
    else:
        rospy.loginfo("no map found")
        return None


def dilate_map(mapdata, padding):
    """
    dilates the map
    :param mapdata [OccupancyGrid] The map data.
    :param padding [int]           The number of cells around the obstacles.
    :return        [OccupancyGrid], [GridCells] The dilated map, added cells
    """
    new_map = OccupancyGrid()
    new_map.info = mapdata.info
    new_map.data = [-1] * (new_map.info.width * new_map.info.height)
    gridcells_msg = GridCells()
    gridcells_msg.header.frame_id = rospy.get_namespace() + "map"
    gridcells_msg.cell_width = mapdata.info.resolution
    gridcells_msg.cell_height = mapdata.info.resolution
    for x in range(mapdata.info.width):
        for y in range(mapdata.info.height):  # iterate through all cells
            if not is_cell_walkable(mapdata, x, y):  # if we find an obstacle
                if not is_cell_unknown(mapdata, x, y):
                    gridcells_msg.cells.append(grid_to_world(mapdata, x, y))
                    new_map.data[grid_to_index(new_map, x, y)] = 100  # mark it unwalkable
                    for dx in range(-1 * padding, padding + 1):
                        for dy in range(-1 * padding, padding + 1):  # iterate through the cells around it
                            if 0 <= x + dx < new_map.info.width and 0 <= y + dy < new_map.info.height:
                                gridcells_msg.cells.append(grid_to_world(mapdata, x + dx, y + dy))
                                new_map.data[grid_to_index(new_map, x + dx, y + dy)] = 100  # mark it unwalkable
                else:
                    if not is_cell_occupied(new_map, x, y):
                        new_map.data[grid_to_index(new_map, x, y)] = -2  # mark it unknown
                        for dx in range(-1 * padding, padding + 1):
                            for dy in range(-1 * padding, padding + 1):  # iterate through the cells around it
                                if not is_cell_occupied(new_map, x + dx,
                                                        y + dy) and 0 <= x + dx < new_map.info.width and \
                                                        0 <= y + dy < new_map.info.height:  # if that cell is currently walkable
                                    new_map.data[grid_to_index(new_map, x + dx, y + dy)] = -2  # mark it unknown
            else:
                if new_map.data[grid_to_index(new_map, x, y)] == -1:
                    new_map.data[grid_to_index(new_map, x, y)] = 0
    return new_map, gridcells_msg


def erode_map(mapdata):
    """
    removes an unwalkable square if it is not surrounded by unwalkable cells
    param mapdata: [OccupancyGrid] thresholded map
    :return: [OccupancyGrid] thresholded map
    """
    new_map = OccupancyGrid()
    new_map.info = mapdata.info
    for x in range(mapdata.info.width):
        for y in range(mapdata.info.height):  # iterate through all cells
            if not is_cell_walkable(mapdata, x, y):  # if we find an obstacle
                if len(neighbors_of_8()) == 8:
                    new_map.data[
                        grid_to_index(mapdata, x, y)] = 100  # mark it unwalkable
    return new_map