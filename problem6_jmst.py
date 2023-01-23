import params, robot_handler
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def jmst_alg(adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
  lowest_node_index = -1
  lowest_cost = max_memory
  memory_to_map = 0
  # calculate how much memory the robot is mapping this round
  for node_index in nodes_to_visit:
    if(nodes[node_index] != robot_handler.Robot_State.MAPPED): # change this if you want to count previously mapped nodes
      memory_to_map += node_memory[node_index]
  # calculate next node based on mst approximation with prim's algorithm
  for node_index in range(len(node_memory)):
    # using prim's algorithm to approx the j-mst
    if (nodes[node_index] == robot_handler.Robot_State.NOT_CLAIMED and lowest_cost > node_memory[node_index] and
         memory_to_map + node_memory[node_index] <= max_memory):
      lowest_cost = node_memory[node_index]
      lowest_node_index = node_index
  
  if(lowest_node_index >= 0):
    return lowest_node_index, False
  else:
    return None, True

robot_handler.generate()
robot_handler.run_all_robots(jmst_alg)