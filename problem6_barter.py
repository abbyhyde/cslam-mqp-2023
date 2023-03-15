import params, robot_handler
import statistics, copy, numpy
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def calculate_mem_used(nodes_visited, node_memory):
    memory_used = 0
    for node_index in nodes_visited: # calcuates used memory
        if(nodes[node_index] != robot_handler.Robot_State.MAPPED): # change this if you want to count previously mapped nodes
            memory_used += node_memory[node_index]
    return memory_used

def pick_new_nodes(curr_nodes, ordered_indexes, node_indexes, node_memory):
    for index in ordered_indexes:
        if(nodes[node_indexes[index]] == robot_handler.Robot_State.NOT_CLAIMED 
           and calculate_mem_used(curr_nodes) + node_memory[node_indexes[index]] <= params.memory): #greedy add unclaimed nodes
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
        if((nodes[node_index] == robot_handler.Robot_State.NOT_CLAIMED)  # or nodes[node_index] == robot_handler.Robot_State.CLAIMED) 
           and memory_to_map + node_memory[node_index] < max_memory and node_index not in nodes_to_visit):
            available_node_memory_list.append(-node_memory[node_index])
            available_node_index_list.append(node_index)
    greatest_node_memory_indexes = numpy.argsort(available_node_memory_list).tolist()
    while(len(greatest_node_memory_indexes) > 0):
        greatest_node_index = available_node_index_list[greatest_node_memory_indexes.pop(0)]
        
        # then we have a good node!
        # 2 possibilities node is clamed by robot a or not
        # loop through each robot to determine if the current node is in one of the robot's fields
        robot_list = robot_handler.robots
        found = False
        old_robot = -1
        for i in range(len(robot_list)):
            cur_robot_nodes = robot_list[i].nodes_to_visit
            for a in cur_robot_nodes:
                if (a == greatest_node_index):
                    found = True
                    old_robot = i
                    break
        
        # if found try a trade
        if (found):
            if (len(robot_list[old_robot].nodes_to_visit) > 1 and robot_list[old_robot].inAuction):
                b_prime_nodes = copy.deepcopy(nodes_to_visit)
                b_prime_nodes.append(greatest_node_index)
                a_prime_nodes = copy.deepcopy(robot_list[old_robot].nodes_to_visit)
                a_prime_nodes.remove(greatest_node_index)
                # generate new paths for the robots based on new node lists
                # get memory left and new paths
                a_prime_nodes = pick_new_nodes(a_prime_nodes, greatest_node_index, available_node_index_list, node_memory)
                a_prime_mem = calculate_mem_used(a_prime_nodes, node_memory)
                b_prime_mem = calculate_mem_used(b_prime_nodes, node_memory)

                # compare if a' and currpath+ new node is better that a and currpath
                    # calculate length of each new path
                    # calculate variance in length of paths
                alpha = 1
                delta = 1
                new_time = calculate_time_to_map(a_prime_nodes, node_memory)
                old_time = calculate_time_to_map(robot_list[old_robot].nodes_to_visit, node_memory)
                b_time = calculate_time_to_map(b_prime_nodes, node_memory)
                # print(str(a_prime_mem) + " " + str(b_prime_mem) + " " + str(new_time) + " " + str(old_time))
                # print(str(delta/a_prime_mem) + " " + str(delta/calculate_mem_used(robot_list[old_robot].nodes_to_visit, node_memory)))
                b_prime_score = (alpha*(b_time) + (delta/b_prime_mem))
                new_score = (alpha*(new_time)) + (delta/a_prime_mem)
                old_score = (alpha*(old_time)) + (delta/calculate_mem_used(robot_list[old_robot].nodes_to_visit, node_memory))
                # go through with trade if a'.length <= a.length,  
                # decreases or equals variance in length in paths, 
                # a.memoryleft >= a'.memoryleft #possible margin
                # use meta heuristics to weight each decision -> make constants for each qualifier and sum together to get score
                # print(str(id) + " " + str(old_robot))
                # print(str(b_prime_score) + " " + str(new_score) + " " + str(old_score))
                if (new_score < old_score and old_robot < id):
                    # remove node from old robot
                    old_robot_obj = robot_handler.robots[old_robot]
                    old_robot_obj.nodes_to_visit.remove(greatest_node_index)
                    old_robot_obj.nodes_to_visit = a_prime_nodes
                    # print("switching node " + str(greatest_node_index))
                else:
                    continue
            else:
                continue
        return greatest_node_index, False, 0
    return None, True, memory_to_map
    
# robot that was stolen from doesn't take new nodes for now

def main():
    robot_handler.generate()
    robot_handler.run_all_robots(barter_alg)

if __name__ == "__main__":
    main()