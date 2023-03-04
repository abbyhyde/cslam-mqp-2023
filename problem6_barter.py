import params, robot_handler
import statistics
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def generate_new_path(robot_nodes, adj_grid, node_memory):
    # this section copied from problem 1 greedy
    at_end = False
    nodes_visited = [0]
    highest_cost = 0
    mem_used = []
    memory_left = params.memory
    while (not at_end):
        possible_next_nodes = []
        # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
        for j in nodes_visited:
            for i in range(0,robot_nodes):
                if((adj_grid[j][i] == 1) and (i not in nodes_visited) and (j != i) and node_memory[i] <= memory_left):
                    # add number to array
                    possible_next_nodes.append(i)
        
        # if robot has enough memory to go to the selected node and robot has not visited it already
        if (len(possible_next_nodes) > 0):
            for i in possible_next_nodes:
                if(highest_cost < node_memory[i]):
                    highest_cost = node_memory[i]
                    next_node = i
            nodes_visited.append(next_node)
            memory_left -= node_memory[next_node]
            highest_cost = 0
        else:
            at_end = True
            #print(str(robot_memory - memory_left)+ " " + str((time.monotonic_ns()-start)/1000000))
            # print("Not enough memory left, done with path at node " + str(current_node))
            # print("Available memory left: " + str(memory_left))
            # print(adj_grid[current_node])
            mem_used.append(robot_memory - memory_left)
    return nodes_visited, memory_left

def barter_alg(adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
    # greedly choose a node 
    greatest_node_memory = -1
    greatest_node_index = -1
    memory_to_map = 0
    for node_index in nodes_to_visit:
        if(nodes[node_index] != robot_handler.Robot_State.MAPPED): # change this if you want to count previously mapped nodes
            memory_to_map += node_memory[node_index]
    for node_index in range(len(node_memory)):
        if(nodes[node_index] == robot_handler.Robot_State.NOT_CLAIMED and memory_to_map + node_memory[node_index] <= max_memory
            and greatest_node_memory < node_memory[node_index]):
            greatest_node_memory = node_memory[node_index]
            greatest_node_index = node_index
    if(greatest_node_index >= 0):
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
        
        # if not, add node to nodes_to_visit
        if (not found):
            nodes_to_visit.append(greatest_node_index)
        else: 
            b_prime_nodes = nodes_to_visit
            b_prime_nodes.append(greatest_node_index)
            a_prime_nodes = cur_robot_nodes
            a_prime_nodes.remove(greatest_node_index)
            # generate new paths for the robots based on new node lists
            # get memory left and new paths
            a_prime_mem, a_prime = generate_new_path(a_prime_nodes, adj_grid, node_memory)
            b_prime_mem, b_prime = generate_new_path(b_prime_nodes, adj_grid, node_memory)

            # compare if a' and currpath+ new node is better that a and currpath
                # calculate length of each new path
                # calculate variance in length of paths?
            alpha = 0.5
            beta = 0.5
            delta = 0.5
            new_score = (alpha*(len(a_prime)+len(b_prime))) + (beta*new_variance) + (delta*new_memory)
            old_score = (alpha*(len(cur_robot_nodes)+len(nodes_to_visit))) + (beta*old_variance) + (delta*old_memory)
            # go through with trade if a'.length <= a.length,  
            # decreases or equals variance in length in paths, 
            # a.memoryleft >= a'.memoryleft #possible margin
            # use meta heuristics to weight each decision -> make constants for each qualifier and sum together to get score
            if (new_score > old_score):
                # add node to nodes_to_visit on new robot
                nodes_to_visit.append(greatest_node_index)
                # remove node from old robot
                old_robot_obj = robot_handler.robots[old_robot]
                old_robot_obj.nodes_to_visit.remove(greatest_node_index)
    return nodes_to_visit
    
# robot that was stolen from doesn't take new nodes for now

def main():
    robot_handler.generate()
    robot_handler.run_all_robots(barter_alg) # has to be changed but idk to what

if __name__ == "__main__":
    main()