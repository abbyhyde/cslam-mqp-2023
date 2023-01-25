import random, math, params, robot_handler
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def uniform_alg(adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
    # print("start")
    next_node_index = -1
    memory_to_map = 0
    # print(nodes_to_visit)
    if (len(nodes_to_visit) < 1):
        # print("EXITING")
        return None, False
    # calculate how much memory the robot is mapping this round
    for node_index in nodes_to_visit:
        if(nodes[node_index] != robot_handler.Robot_State.MAPPED): # change this if you want to count previously mapped nodes
            memory_to_map += node_memory[node_index]
    # calculate next node based on randomly picking one and checking if it meets qualifications
    possible_nodes = []
    for node_index in range(len(nodes)):
        if (nodes[node_index] == robot_handler.Robot_State.NOT_CLAIMED and memory_to_map + node_memory[node_index] <= max_memory):
            possible_nodes.append(node_index)
    if(len(possible_nodes) > 0):
        next_node_index = possible_nodes[math.floor(random.random()*len(possible_nodes))]
        return next_node_index, False
    else:
        return None, True

robot_handler.generate()
robot_handler.run_all_robots(uniform_alg)