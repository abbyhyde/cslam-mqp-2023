import random, numpy, math, time
import params

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

#settings params from params.py
nodes = params.nodes
robot_memory = params.memory
robots = params.robots
random.seed(params.seed)
for i in range(0,20):
  adj_grid = numpy.eye(nodes, nodes, 0, int)
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
  # print(node_memory)

  # perform bfs to check whether the graph is connected
  current_node_index = 0
  nodes_checked = []
  nodes_checked.append(current_node_index)
  queue = []
  queue.append(current_node_index)
  while queue:
    current_node_index = queue.pop(0)
    # print(current_node_index)
    for i in adj_grid[current_node_index]:
      if i not in nodes_checked and adj_grid[current_node_index][i] == 1:
        nodes_checked.append(i)
        queue.append(i)
  
  # now we check which nodes were not covered by bfs
  for j in range(nodes):
    if j not in nodes_checked:
      # connect the node to a random node in the main part of the graph
      connection_index = math.floor(random.random() * len(nodes_checked))
      adj_grid[connection_index][j] = 1
      adj_grid[j][connection_index] = 1
      # print("connected " + str(j) + " to " + str(connection_index))


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
      print(str(robot_memory - memory_left)+ " " + str((time.monotonic_ns()-start)/1000000))
      # print("Not enough memory left, done with path at node " + str(current_node))
      # print("Available memory left: " + str(memory_left))
      # print(adj_grid[current_node])
    # print("Path: " + str(nodes_visited))