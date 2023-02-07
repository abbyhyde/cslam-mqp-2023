import random, numpy, math, time, robot_handler
import params
import sample_adj_grids_and_memories as sample

def isTravelable(path):
  nodes_visit = []
  nodes_reachable = []
  for i in range(nodes):
        if (math.floor(path[1] / ( 2**i )) % 2 == 1):
          nodes_visit.append(i)
  if nodes_visit[0] != 0:
      return False
  else:
    nodes_reachable.append(nodes_visit[0])
  for curr_node in nodes_reachable:
    for test_node in nodes_visit:
      if adj_grid[curr_node][test_node] == 1 and test_node not in nodes_reachable:
        nodes_reachable.append(test_node)
  if len(nodes_reachable) == len(nodes_visit):
    return True
  return False


mem_used = []
for i in range(30):
#settings params from params.py
  nodes = params.num_nodes
  robot_memory = params.memory
  robots = params.robots
  

  adj_grid = sample.adj_grids[i]
  node_memory = sample.node_memories[i]
        
  # greedy algorithm
  # adds the highest possible cost to explore the longest as possible

  # mapping the path
  # make that the next node
  at_end = False
  highest_cost = 0
  memory_left = robot_memory
  next_node = 0
  nodes_visited = []
  possible_next_nodes = []
  nodes_visited.append(next_node)
  # while not at the end or no more possible options
  start = time.monotonic_ns()
  while (not at_end):
    # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
    possible_next_nodes = []
    # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
    for j in nodes_visited:
      for i in range(0,nodes):
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
      #print("Path: " + str(nodes_visited))
    
for value in mem_used:
  print(value)