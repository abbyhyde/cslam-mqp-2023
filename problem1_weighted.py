import random, numpy, math, time
import params

def calculateNodeWeight(node_weights, index):
  total = 0
  for node_weight in node_weights:
    total += node_weight
  return node_weights[index]/total

#settings params from params.py
nodes = params.nodes
robot_memory = params.memory
robots = params.robots
for i in range(0,20):
  adj_grid = numpy.eye(nodes)
  node_memory = numpy.empty(nodes)
  prob_connection = 0.5
  for i in range(0,nodes): #assumes the starting node is only connected to the first node add the ability for other nodes to be reached from the start
    rfloat = random.random()
    node_memory[i] = (robot_memory/1.5)*rfloat
    for j in range(i,nodes):
      if(i == j or random.random() < prob_connection):
        adj_grid[i][j] = 1
        adj_grid[j][i] = 1
  # print(adj_grid)
  #print(node_memory)

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