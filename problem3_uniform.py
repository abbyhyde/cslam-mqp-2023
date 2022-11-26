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

# some way to store paths that happened before -> array of strings
for i in range(0,1):  # the number of times we're running the experiment
  adj_grid = numpy.eye(nodes)
  node_memory = numpy.empty(nodes)
  nodes_mapped = numpy.empty(nodes)
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

  # uniform probabilistic algorithm
  # gives each adj node an equal chance and picks one
  nodes_mapped[0] = 1
  # while not at the end or no more possible options
  start = time.monotonic_ns()
  def isDone():
    for i in nodes_mapped:
        if(i != 1):
            return True
    return False

  robot = 0
  while(isDone()):
    # mapping the path
    at_end = False
    expand = False
    highest_cost = 0
    memory_left = robot_memory
    next_node = 0
    current_node = 0
    nodes_visited = []
    possible_next_nodes = []
    pick = 0
    nodes_visited.append(current_node)
    # while not at the end or no more possible options
    start = time.monotonic_ns()
    while (not at_end):
      possible_next_nodes = []
      # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
      for j in nodes_visited:
        for i in range(0,nodes):
          if((adj_grid[j][i] == 1) and (i not in nodes_visited) and (j != i) and node_memory[i] <= memory_left and (i not in possible_next_nodes)):
            # add number to array
            possible_next_nodes.append(i)
      # print(nodes_visited)
      # print(nodes_mapped)
      # print(possible_next_nodes)
      
      # if robot has enough memory to go to the selected node and robot has not visited it already
      """
      if there are possible next nodes:
        while there are possible next nodes
          randomly pick one
          if the node memory is not the highest cost
            make it the next node
          else
            remove from possible next nodes list
        here if ran out of nodes
        pick one from the visited nodes and make it the next node
      """
      size = len(possible_next_nodes)
      if (size > 0):
        while(size > 0):
          next_node = possible_next_nodes[math.floor(random.random()*size)]
          # print(str(size) + "   " + str(next_node))
          if(highest_cost < node_memory[next_node] and nodes_mapped[next_node] != 1):
            highest_cost = node_memory[next_node]
            break
          else:    
            # print("removing node " + str(next_node))
            possible_next_nodes.remove(next_node)
            size = len(possible_next_nodes)
        nodes_visited.append(next_node)
        nodes_mapped[next_node] = 1
        memory_left -= node_memory[next_node]
        highest_cost = 0
        # print("remaining memory: " + str(memory_left))
      elif(not expand):
        expand = True
        for i in range(1, len(nodes_mapped)):
            if (i not in nodes_visited and nodes_mapped[i] == 1):
                nodes_visited.append(i)
      else:
        at_end = True
        print("robot-" + str(robot)+ " " + str(robot_memory - memory_left)+ " " + str((time.monotonic_ns()-start)/1000000))
        print(nodes_mapped)
        robot += 1
        # print("Not enough memory left, done with path at node " + str(current_node))
        # print("Available memory left: " + str(memory_left))
        # print(adj_grid[current_node])
        print("Path: " + str(nodes_visited))
        input()