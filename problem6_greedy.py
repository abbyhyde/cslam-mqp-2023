import params, robot_handler
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def greedy_alg(adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
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
        return greatest_node_index, False
    else:
        return None, True


robot_handler.generate()
robot_handler.run_all_robots(greedy_alg)