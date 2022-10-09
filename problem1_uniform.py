import random, numpy, math

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

nodes = 10
robot_memory = 10
adj_grid = numpy.eye(nodes)
node_memory = numpy.empty(nodes)
prob_connection = 0.5
for i in range(0,nodes): #assumes the starting node is only connected to the first node add the ability for other nodes to be reached from the start
  node_memory[i] = (robot_memory/1.5)*random.random()
  for j in range(i,nodes):
    if(i == j or random.random() < prob_connection):
      adj_grid[i][j] = 1
      adj_grid[j][i] = 1
print(adj_grid)
print(node_memory)

# uniform probabilistic algorithm
# gives each adj node an equal chance and picks one

# mapping the path
at_end = False
memory_left = robot_memory
next_node = 0
current_node = 0
nodes_visited = []
possible_next_nodes = []
pick = 0
nodes_visited.append(current_node)
# while not at the end or no more possible options
while (not at_end):
  # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
  for i in range(0,nodes):
    if((adj_grid[current_node][i] == 1) and (i not in nodes_visited) and (current_node != i)):
      # add number to array
      possible_next_nodes.append(i)
  
  # pick next node
  next_node = numpy.random.choice(possible_next_nodes)
  
  # if robot has enough memory to go to the selected node and robot has not visited it already
  if (node_memory[next_node] < memory_left and next_node not in nodes_visited):
    nodes_visited.append(next_node)
    current_node = next_node
    memory_left -= node_memory[next_node]
    print("Next node: " + str(next_node) + ", has cost of " + str(node_memory[next_node]))
    print("Available memory left: " + str(memory_left))
  else:
    at_end = True
    print("Done with path at node " + str(current_node))
    print(adj_grid[current_node])
    print("Path: " + str(nodes_visited))