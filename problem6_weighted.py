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

    possible_nodes = []
    for node_index in range(len(nodes)):
        if (nodes[node_index] == robot_handler.Robot_State.NOT_CLAIMED and memory_to_map + node_memory[node_index] <= max_memory):
            possible_nodes.append(node_index)

    if(len(possible_nodes) > 0):
      choice = random.random() - calculateNodeWeight(node_memory, possible_nodes[0]) # randomize how much weight we'll go through in order
      next_node_index = 0
      next_node = possible_nodes[next_node_index]
      while choice >= 0 and next_node_index < len(possible_nodes)-1: # while there's still stuff to go through
        next_node_index = 1 + next_node_index
        next_node = possible_nodes[next_node_index]
        choice -= calculateNodeWeight(node_memory, next_node) # remove the next amount of stuff to go through
      return next_node, False
    else:
      return None, True

    

robot_handler.generate()
robot_handler.run_all_robots(weighted_alg)