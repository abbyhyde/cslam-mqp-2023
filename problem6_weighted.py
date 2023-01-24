import random, math, params, robot_handler
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def calculateNodeWeight(node_weights, index):
  total = 0
  for node_weight in node_weights:
    total += node_weight
  return node_weights[index]/total

def weighted_alg(adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
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
    # nodes_checked = []
    # node_found = False
    # while (not node_found):
    #     next_node_index = random.random() - calculateNodeWeight(node_memory, nodes_to_visit[0])
    #     # print(next_node_index)
    #     if (len(nodes_checked) == len(nodes_to_visit)):
    #         # print("out")
    #         return None, True
    #     elif (nodes[next_node_index] == robot_handler.Robot_State.NOT_CLAIMED 
    #               and memory_to_map + node_memory[next_node_index] <= max_memory):
    #         # print("ok")
    #         node_found = True
    #         return next_node_index, False
    #     elif (next_node_index not in nodes_checked):
    #         # print("added " + str(next_node_index))
    #         nodes_checked.append(next_node_index)
  
    if(next_node_index >= 0):
        print(next_node_index)
        return next_node_index, False
    else:
        return None, True

robot_handler.generate()
robot_handler.run_all_robots(weighted_alg)