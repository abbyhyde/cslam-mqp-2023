import random, numpy, math, time
import params
import sample_adj_grids_and_memories as sample

def calculateNodeWeight(node_weights, index):
  total = 0
  for node_weight in node_weights:
    total += node_weight
  return node_weights[index]/total

mem_used = []
for i in range(100):
#settings params from params.py
  nodes = params.num_nodes
  robot_memory = params.memory
  robots = params.robots
  

  adj_grid = sample.adj_grids[math.ceil(i/30)%30]
  node_memory = sample.node_memories[i%30]


  # weighted probabilistic algorithm
  # gives each adj node an equal chance and picks one

  # mapping the path
  at_end = False
  memory_left = robot_memory
  nodes_visited = [0]
  # while not at the end or no more possible options
  start = time.monotonic_ns()
  while (not at_end):
    possible_next_nodes = []
    # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
    for j in nodes_visited:
      for i in range(0,nodes):
        if((adj_grid[j][i] == 1) and (i not in nodes_visited) and (j != i) and node_memory[i] <= memory_left):
          # add number to array
          possible_next_nodes.append(i)
    

    
    # if robot has enough memory to go to the selected node and robot has not visited it already
    if (len(possible_next_nodes) > 0):
      # pick next node
      choice = random.random() - calculateNodeWeight(node_memory, possible_next_nodes[0])
      next_node_index = 0
      while choice >= 0 and next_node_index < len(possible_next_nodes)-1:
        next_node_index = 1 + next_node_index
        choice -= calculateNodeWeight(node_memory, possible_next_nodes[next_node_index])
      nodes_visited.append(possible_next_nodes[next_node_index])
      memory_left -= node_memory[possible_next_nodes[next_node_index]]
      #print("Next node: " + str(possible_next_nodes[next_node_index]) + ", has cost of " + str(node_memory[possible_next_nodes[next_node_index]]))
      #print("Available memory left: " + str(memory_left))
    else:
      at_end = True
      #print("Path: " + str(nodes_visited))
      print(str(robot_memory - memory_left)+ " " + str((time.monotonic_ns()-start)/1000000))
      mem_used.append(robot_memory - memory_left)
    
for value in mem_used:
  print(value)