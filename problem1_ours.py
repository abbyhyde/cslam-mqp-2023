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


#general idea add power set of node memories
#any thing above max memory get axed, go through the others to find a valid path

#calculate cost of each combo of nodes
start = time.monotonic_ns()
possible_paths = []
for i in range(1,2**nodes):
  cost = 0
  for j in range(nodes):
      if (math.floor(i / ( 2**j )) % 2 == 1):
        cost += node_memory[j]
  if (cost < robot_memory):
      possible_paths.append((cost, i))
possible_paths = sorted(possible_paths, reverse=True)
#print(possible_paths)

for possible_path in possible_paths:
  if(isTravelable(possible_path)):
    print(possible_path)
    nodes_in_path = []
    for i in range(nodes):
        if (math.floor(possible_path[1] / ( 2**i )) % 2 == 1):
          nodes_in_path.append(i)
    print(nodes_in_path)
    break
print((time.monotonic_ns()-start)/1000000)
