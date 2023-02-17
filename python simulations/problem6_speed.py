import params, robot_handler
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def speedy_alg(adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
    least_node_distance = None
    least_node_index = -1
    memory_to_map = 0
    for node_index in nodes_to_visit:
        if(nodes[node_index] != robot_handler.Robot_State.MAPPED): # change this if you want to count previously mapped nodes
            memory_to_map += node_memory[node_index]
    if (nodes[0] == robot_handler.Robot_State.NOT_CLAIMED and len(nodes_to_visit) == 0):
        least_node_index = 0  
    else:
        if 0 not in nodes_to_visit:
            nodes_to_visit.append(0)
        for node_index in range(len(node_memory)):
            if(nodes[node_index] == robot_handler.Robot_State.NOT_CLAIMED and memory_to_map + node_memory[node_index] <= max_memory):
                distance = None
                for j in nodes_to_visit:
                    if(nodes[j] == robot_handler.Robot_State.MAPPED):
                        nodes_between = robot_handler.Robot.bfs(j,node_index)
                        if(nodes_between is not None and (distance is None or distance > len(nodes_between))):
                            distance = len(nodes_between)
                if(distance is not None and (least_node_distance is None or least_node_distance > distance)):
                    least_node_distance = distance
                    least_node_index = node_index
    if(least_node_index >= 0):
        return least_node_index, False, 0
    else:
        return None, True, memory_to_map

def main():
    robot_handler.generate()
    robot_handler.run_all_robots(speedy_alg)

if __name__ == "__main__":
    main()