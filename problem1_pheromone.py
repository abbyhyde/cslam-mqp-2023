import random, numpy, math, threading

def calculateNodeWeight(prev, next):
  total = 0
  for i in range(0, len(node_memory)):
    total += node_memory[i]*edge_pheromone[prev][i]
  return node_memory[next]*edge_pheromone[prev][next]/total

def ant():
  at_end = False
  memory_left = robot_memory
  nodes_visited = [0]
  prev_node = [-1]
  while (not at_end):
    possible_next_nodes = []
    found_from = []
    # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
    for j in nodes_visited:
      for i in range(0,nodes):
        if((adj_grid[j][i] == 1) and (i not in nodes_visited) and (j != i) and node_memory[i] <= memory_left):
          # add number to array
          possible_next_nodes.append(i)
          found_from.append(j)
    
    # if robot has enough memory to go to the selected node and robot has not visited it already
    if (len(possible_next_nodes) > 0):
      # pick next node
      choice = random.random() - calculateNodeWeight(found_from[0], possible_next_nodes[0])
      next_node_index = 0
      while choice >= 0 and next_node_index < len(possible_next_nodes)-1:
        next_node_index = 1 + next_node_index
        choice -= calculateNodeWeight(found_from[next_node_index], possible_next_nodes[next_node_index])
      nodes_visited.append(possible_next_nodes[next_node_index])
      memory_left -= node_memory[possible_next_nodes[next_node_index]]
      prev_node.append(found_from[next_node_index])
    else:
      at_end = True
  if len(nodes_visited) > 1:
    for i in range(1,len(nodes_visited)):
      edge_pheromone[prev_node[i]][nodes_visited[i]] = edge_pheromone[prev_node[i]][nodes_visited[i]] * pheromone_decay + 1 - (memory_left / 10)
  return nodes_visited, memory_left


nodes = 10
robot_memory = 10
adj_grid = numpy.eye(nodes)
node_memory = numpy.empty(nodes)
edge_pheromone = numpy.ones((nodes, nodes))
prob_connection = 0.5
pheromone_decay = 0.75
for i in range(0,nodes): #assumes the starting node is only connected to the first node add the ability for other nodes to be reached from the start
  node_memory[i] = (robot_memory/1.5)*random.random()
  for j in range(i,nodes):
    if(i == j or random.random() < prob_connection):
      adj_grid[i][j] = 1
      adj_grid[j][i] = 1
print(adj_grid)
print(node_memory)

for i in range(0,999):
  n,m = ant()
nodes_visited, memory_left = ant()
print("Path: " + str(nodes_visited))
print("Available memory left: " + str(memory_left))