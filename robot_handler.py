""" 
Generates graph for each runthrough as well as manages the thread for each robot. 
"""
import random
import numpy
import math, time, logging, threading
import params
from enum import Enum

#settings params from params.py
num_nodes = params.num_nodes
robot_memory = params.memory
num_robots = params.robots
random.seed(params.seed)

class Robot_State(Enum):
    NOT_ENCOUNTERED = 0
    NOT_CLAIMED = 1
    CLAIMED = 2
    MAPPED = 3

nodes = []
for i in range(num_nodes):
    nodes.append(Robot_State.NOT_ENCOUNTERED)
nodes[0] = Robot_State.NOT_CLAIMED
adj_grid = numpy.eye(num_nodes, num_nodes, 0, int)
node_memory = numpy.empty(num_nodes)
robots = [] # holds all robot objects

# generates any/all of the graphs for data collection
def generate_graph(gt, pct):
    global adj_grid, node_memory
    sum_node_memory = robot_memory * num_robots * pct # calculate the total amount of memory on the graph
    # print("Total sum of nodes: " + str(sum_node_memory))
    if (gt == 0):
        # print("Building lattice graph...")
        # currently hardcoded for 20 nodes but will be changed at some point
        adj_grid = [[1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
        [1,0,0,0,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0,1,1,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,0],
        [0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0],
        [0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0],
        [0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1],
        [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1]]
    elif (gt == 1):
        # print("Building fully connected graph...")
        adj_grid = [[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]]
    elif (gt == 2):
        # print("Building tree graph... ")
        # currently hardcoded for 20 nodes but will be changed at some point
        adj_grid = [[1,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0],
        [1,1,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,1,1,0,1,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0],
        [1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1]]
    # now assign node memory
    valid = False
    rounds = 0
    while (not valid):
        params.seed += 1
        numpy.random.seed(params.seed)  
        node_memory = (numpy.random.multinomial(sum_node_memory-num_nodes, numpy.ones(num_nodes)/num_nodes, size=1) + numpy.ones(num_nodes))[0] 
        # print(node_memory)
        valid = True
        rounds += 1
        for i in node_memory:
            if (i > 10.5):
                valid = False
        if (rounds > 10):
            valid = True
            print("we gave up and things suck")
    # print(node_memory)

def generate():
    global adj_grid
    adj_grid = numpy.eye(num_nodes, num_nodes, 0, int)
    prob_connection = 0.2
    for i in range(0,num_nodes): #assumes the starting node is only connected to the first node add the ability for other nodes to be reached from the start
        rfloat = random.random()
        node_memory[i] = math.ceil((robot_memory/1.5)*rfloat)
        for j in range(i,num_nodes):
            if(i == j or random.random() < prob_connection):
                adj_grid[i][j] = 1
                adj_grid[j][i] = 1
    # print(adj_grid)
    # print(node_memory)
    # perform bfs to check whether the graph is connected
    current_node_index = 0
    nodes_checked = []
    nodes_checked.append(current_node_index)
    queue = []
    queue.append(current_node_index)
    while queue:
        current_node_index = queue.pop(0)
        # print(current_node_index)
        for i in range(0, num_nodes):
            if i not in nodes_checked and adj_grid[current_node_index][i] == 1:
                nodes_checked.append(i)
                queue.append(i)
    # now we check which nodes were not covered by bfs
    for j in range(num_nodes):
        if j not in nodes_checked:
            # connect the node to a random node in the main part of the graph
            connection_index = math.floor(random.random() * len(nodes_checked))
            adj_grid[nodes_checked[connection_index]][j] = 1
            adj_grid[j][nodes_checked[connection_index]] = 1
            nodes_checked.append(j)
            # print("connected " + str(j) + " to " + str(connection_index))
    return adj_grid, node_memory

def run_next_robot(algorithm,signal,doneSignal):
    threads = []
    # runs all robots
    index = 0
    while (not doneSignal.is_set()):
        if (len(threads) == index):
            #print("     Main: create and start robot " + str(index))
            new_thread = threading.Thread(target=algorithm, args=(index,))
            threads.append(new_thread)
            new_thread.start()
            signal.wait(timeout=60.0)
        elif (threads[index].is_alive()):
            continue
        else:
            #print("     Main: restart robot " + str(index))
            threads[index] = threading.Thread(target=algorithm, args=(index,))
            new_thread = threads[index]
            new_thread.start()
            signal.wait(timeout=60.0)
        index = (index+1) % num_robots

    for index, thread in enumerate(threads):
        thread.join()
        #print("     Main: done with robot " + str(index))


class Robot:
    def __init__(self, id, alg, max_memory):
        self.id = id
        self.algorithm = alg
        self.max_memory = max_memory
        self.memory_usage = 0
        self.nodes_to_visit = []
        self.nodes_visited = []
        self.memory_left_to_map = 0
        self.curr_node = -1 # -1 means outside the graph, robots always enter into node 0
        self.path = []
        self.path.append(0)
        self.inAuction = False
    
    def __str__(self):
        return "id: " + str(self.id) + " memory usage: " + str(self.memory_usage) + " nodes to visit: " + str(self.nodes_to_visit)
        
    def act(self): # warning does not add new nodes it discovers yet if they can fit needs to do this for it to work
        if(self.curr_node == -1 and len(self.nodes_to_visit) == 0): #if the robot is at the base with nothing to do
            self.memory_left_to_map = 0
            self.memory_usage = 0
            return True #returns the true that it wants to be included in the auction
        elif(self.memory_left_to_map > 0 and self.memory_usage + node_memory[self.curr_node] < self.max_memory): #if the robot is still mapping a node
            self.memory_left_to_map -= 1
            self.memory_usage += 1
            return False #does not want to be included in the auction
        else: #robot moves
            if(self.curr_node == -1): #move onto the graph (node 0)
                self.nodes_visited = []
                self.curr_node = 0
                if (nodes[self.curr_node] != Robot_State.MAPPED):
                    self.memory_left_to_map = node_memory[self.curr_node]
                return False
            elif len(self.nodes_to_visit) > 0:
                next_node = Robot.bfs(self.curr_node, self.nodes_to_visit[0])[-1] #should be new next node on the way to the next target node
                if(next_node == self.nodes_to_visit[0]):
                    self.nodes_to_visit.remove(next_node)
                    self.nodes_visited.append(next_node)
                    self.memory_left_to_map = node_memory[next_node]
                self.curr_node = next_node
                self.path.append(next_node)
            elif(self.curr_node != 0):
                self.curr_node  = Robot.bfs(self.curr_node, 0)[-1]
                self.path.append(self.curr_node)
            else:
                self.curr_node = -1 #if no escape edges we must be done traversing
                self.done = True

            return False

    def pick(self):
        new_node_to_visit, result, memory_usage = self.algorithm(self.id, adj_grid, node_memory, self.nodes_to_visit, self.max_memory, nodes)
        # print("picked node " + str(new_node_to_visit))
        if not result:
            self.nodes_to_visit.append(new_node_to_visit)
            #order nodes better?
        
        return result, new_node_to_visit, memory_usage

    def bfs(start, end): # tries to find a path from start to end using known nodes
        current_node_index = start
        nodes_checked = []
        nodes_checked.append(current_node_index)
        connections = dict()
        queue = []
        queue.append(current_node_index)
        while queue:
            current_node_index = queue.pop(0)
            for i in range(len(nodes)):
                if (i not in nodes_checked and adj_grid[current_node_index][i] == 1 and nodes[i] == Robot_State.MAPPED) or (i == end and adj_grid[current_node_index][i] == 1):
                    if i != end:
                        nodes_checked.append(i)
                        queue.append(i)
                        connections[i] = current_node_index
                    else:
                        nodes_between = [i]
                        while current_node_index != start:
                            nodes_between.append(current_node_index)
                            current_node_index = connections[current_node_index]
                        return nodes_between
        return None


def done():
    # run through all node states and check if they've been mapped
    for node in nodes:
        if node != Robot_State.MAPPED:
            return False
    return True

def holdAuction(robots_in_auction):
    # for every node in the list, assign a robot
    # unless every robot has possible memory filled
    auction_index = 0
    robots_full = 0 
    memory_usage = []
    # assign nodes as long as there are nodes left to assign and there are robots that don't have full memory
    unclaimed_nodes = False
    for node in nodes:
        if node == Robot_State.NOT_CLAIMED:
            unclaimed_nodes = True

    while(robots_full != math.pow(2,len(robots_in_auction))-1 and unclaimed_nodes):
        full, node_claimed, new_memory_usage = robots[robots_in_auction[auction_index]].pick()
        if full:
            if(new_memory_usage != 0):
                memory_usage.append(new_memory_usage)
            robots_full = robots_full | (1<<auction_index)
        if node_claimed is not None:
            nodes[node_claimed] = Robot_State.CLAIMED
        auction_index = (auction_index + 1) % len(robots_in_auction)
    return memory_usage

def count_unclaimed():
    count = 0
    for node in nodes:
        if node == Robot_State.NOT_CLAIMED:
            count += 1
    return count

def run_all_robots(algorithm):
    #reset
    #print(adj_grid)
    global num_robots 
    num_robots = params.robots
    global nodes 
    nodes = []
    for i in range(num_nodes):
        nodes.append(Robot_State.NOT_ENCOUNTERED)
    nodes[0] = Robot_State.NOT_CLAIMED
    global robots 
    robots = []
    for r in range(num_robots):
        # create robot and append to robots
        newRobot = Robot(r, algorithm, params.memory)
        robots.append(newRobot)
    robots_in_auction = range(num_robots)
    round = 0
    memory_usage = []
    while(not done()):
        if (round > 3000):
            print("we done fucked up spock")
            break
        # print("round: "+ str(round))
        round += 1
        # print("nodes: " + str (nodes))
        robot_string = ""
        for robot in robots:
            robot_string = robot_string + "\n" + str(robot)
        # print("robots: " + robot_string)
        new_usage = holdAuction(robots_in_auction)
        for robot in robots:
            robot.inAuction = False
        for value in new_usage:
            memory_usage.append(value)
        robots_in_auction = []
        for i in range(num_robots):
            back = robots[i].act()
            # if robot is done, parse all nodes from the robot and label accordingly
            if(back):
                robots_in_auction.append(i)
                robots[i].inAuction = True
                for node in robots[i].nodes_to_visit:
                    nodes[node] = Robot_State.NOT_CLAIMED
                    robots[i].nodes_to_visit.remove(node)
                for node in robots[i].nodes_visited:
                    nodes[node] = Robot_State.MAPPED # the neighbors of this need to be added to not_claimed
                    for j in range(len(nodes)):
                        if (adj_grid[j][node] == 1 and nodes[j] == Robot_State.NOT_ENCOUNTERED):
                            nodes[j] = Robot_State.NOT_CLAIMED # adding the new nodes at the frontier to not being claimed
    # print("round: "+ str(round))
    round += 1
    # print("nodes: " + str (nodes))
    robot_string = ""
    for robot in robots:
        robot_string = robot_string + "\n" + str(robot)
    # print("robots: " + robot_string)
    return round, memory_usage