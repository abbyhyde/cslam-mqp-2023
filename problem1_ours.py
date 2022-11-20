import random, numpy, math, time
import seed

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
print(adj_grid)
print(node_memory)

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
