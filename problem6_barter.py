"""
Barter Algorithm - every time it acts upon the greedy algorithm, it also checks to see if nodes can be switched to the current
robot to make the paths more efficient. Calculates the score for the robot that currently has the node and for the robot that is 
requesting the node. Score calculated based on the path length and memory needed for the path, weighted with two constants. Nodes
can only be swapped from robots with higher ids than the robot with the node to prevent infinite swaps. 
"""
import params, robot_handler
import statistics, copy, numpy
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def calculate_mem_used(nodes_visited, node_memory):
    memory_used = 0
    for node_index in nodes_visited: # calculates used memory
        if(nodes[node_index] != robot_handler.Robot_State.MAPPED): # change this if you want to count previously mapped nodes
            memory_used += node_memory[node_index]
    return memory_used

def pick_new_nodes(curr_nodes, ordered_indexes, node_indexes, node_memory):
    for index in ordered_indexes:
        if(nodes[node_indexes[index]] == robot_handler.Robot_State.NOT_CLAIMED 
           and calculate_mem_used(curr_nodes) + node_memory[node_indexes[index]] <= params.memory):
            curr_nodes.append(node_indexes[index])
    return curr_nodes

def calculate_time_to_map(curr_nodes, node_memory):
    time_to_map = calculate_mem_used(curr_nodes, node_memory)
    start_node = 0
    if(len(curr_nodes) != 0):
        for node in curr_nodes:
            time_to_map += len(robot_handler.Robot.bfs(start_node, node))
            start_node = node
        time_to_map += len(robot_handler.Robot.bfs(curr_nodes[-1], 0))
    return time_to_map

def barter_alg(id, adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
    # greedly choose a node 
    available_node_memory_list = []
    available_node_index_list = []
    memory_to_map = calculate_mem_used(nodes_to_visit, node_memory)
    for node_index in range(len(node_memory)):
        if((nodes[node_index] == robot_handler.Robot_State.NOT_CLAIMED or nodes[node_index] == robot_handler.Robot_State.CLAIMED) 
           and memory_to_map + node_memory[node_index] <= max_memory and node_index not in nodes_to_visit):
            available_node_memory_list.append(-node_memory[node_index])
            available_node_index_list.append(node_index)
    greatest_node_memory_indexes = numpy.argsort(available_node_memory_list).tolist()
    while(len(greatest_node_memory_indexes) > 0):
        # get the first possible node to swap
        greatest_node_index = available_node_index_list[greatest_node_memory_indexes.pop(0)]
        # print(greatest_node_index)
        
        # we have a good node!
        # 2 possibilities node is clamed by robot a or not
        # robot a = old robot = robot the node is currently with
        # robot b = new robot = robot trying to barter for the node
        # loop through each robot to determine if the current node is in one of the robot's fields
        robot_list = robot_handler.robots
        found = False
        old_robot = -1
        for i in range(len(robot_list)):
            cur_robot_nodes = robot_list[i].nodes_to_visit
            # print(cur_robot_nodes)
            for a in cur_robot_nodes:
                if (a == greatest_node_index):
                    found = True
                    old_robot = i
                    break
        # if found try a trade
        if (found):
            if (len(robot_list[old_robot].nodes_to_visit) > 1 and robot_list[old_robot].inAuction):
                new_robot_nodes = copy.deepcopy(nodes_to_visit)
                new_robot_nodes.append(greatest_node_index)
                old_robot_nodes = copy.deepcopy(robot_list[old_robot].nodes_to_visit)
                old_robot_nodes.remove(greatest_node_index)
                # generate new paths for the robots based on new node lists
                # get memory left and new paths
                old_robot_nodes = pick_new_nodes(old_robot_nodes, greatest_node_memory_indexes, available_node_index_list, node_memory)
                old_robot_mem = calculate_mem_used(old_robot_nodes, node_memory)
                new_robot_mem = calculate_mem_used(new_robot_nodes, node_memory)

                # compare if old robot's new path (w/o node) and new robot's new path with new node is better
                # constants to weight the time it takes to map a path and the amount of memory used for a path
                alpha = 1
                delta = 1
                new_time = calculate_time_to_map(old_robot_nodes, node_memory) # time for old robot to map without the node
                old_time = calculate_time_to_map(robot_list[old_robot].nodes_to_visit, node_memory) # time for old robot to map with the node
                new_robot_time = calculate_time_to_map(new_robot_nodes, node_memory) # time for new robot to map with the node
                # print(str(old_robot_mem) + " " + str(new_robot_mem) + " " + str(new_time) + " " + str(old_time))
                # print(str(delta/old_robot_mem) + " " + str(delta/calculate_mem_used(robot_list[old_robot].nodes_to_visit, node_memory)))
                new_robot_score = (alpha*(new_robot_time) + (delta/new_robot_mem))
                new_score = (alpha*(new_time)) + (delta/old_robot_mem)
                old_score = (alpha*(old_time)) + (delta/calculate_mem_used(robot_list[old_robot].nodes_to_visit, node_memory))
                # print(str(id) + " " + str(old_robot))
                # print(str(new_robot_score) + " " + str(new_score) + " " + str(old_score))
                # check if it's worth it to swap, and swaps can only occur from a robot with a lower id to one with a higher id
                if (new_score < old_score and old_robot < id):
                    # remove node from old robot
                    old_robot_obj = robot_handler.robots[old_robot]
                    old_robot_obj.nodes_to_visit.remove(greatest_node_index)
                    old_robot_obj.nodes_to_visit = old_robot_nodes
                    # print("switching node " + str(greatest_node_index))
                else:
                    continue
            else:
                continue
        return greatest_node_index, False, 0
    return None, True, memory_to_map
    
def main():
    robot_handler.generate()
    robot_handler.run_all_robots(barter_alg)

if __name__ == "__main__":
    main()

#previous planning: 
# go through with trade if a'.length <= a.length,  
# decreases or equals variance in length in paths, 
# a.memoryleft >= a'.memoryleft #possible margin
# use meta heuristics to weight each decision -> make constants for each qualifier and sum together to get score
# robot that was stolen from doesn't take new nodes for now