import random, numpy, math, time, threading
import params, robot_handler

#settings params from params.py
nodes = params.nodes
robot_memory = params.memory
robots = params.robots
random.seed(params.seed)

nodes_mapped = numpy.empty(nodes)
adj_grid, node_memory = robot_handler.generate()
signal = threading.Event()
doneSignal = threading.Event()

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

def isDone():
  for i in nodes_mapped:
      if(i != 1):
          return True
  return False

def uniform_alg(index):
  # will contain the entire uniform alg
  # uniform probabilistic algorithm
  # gives each adj node an equal chance and picks one
  nodes_mapped[0] = 1
  # while not at the end or no more possible options
  start = time.monotonic_ns()

  # mapping the path
  at_end = False
  expand = False
  memory_left = robot_memory
  next_node = 0
  current_node = 0
  nodes_visited = []
  possible_next_nodes = []
  nodes_visited.append(current_node)
  # while not at the end or no more possible options
  start = time.monotonic_ns()
  while (not at_end):
    # print("Robot " + str(index) + " starting loop " + str(at_end))
    possible_next_nodes = []
    # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
    for j in nodes_visited:
      for i in range(0,nodes):
        if((adj_grid[j][i] == 1) and (i not in nodes_visited) and (j != i) and node_memory[i] <= memory_left and (i not in possible_next_nodes)):
          # add number to array
          possible_next_nodes.append(i)
    
    # if robot has enough memory to go to the selected node and robot has not visited it already
    if (len(possible_next_nodes) > 0):
      # print("Robot " + str(index) + " where we want")
      next_node = possible_next_nodes[math.floor(random.random()*len(possible_next_nodes))]
      nodes_visited.append(next_node)
      memory_left -= node_memory[next_node]
      highest_cost = 0
      if (signal.is_set() == False): 
        signal.set()
    elif(not expand):
      expand = True
      for i in range(1, len(nodes_mapped)):
          if (i not in nodes_visited and nodes_mapped[i] == 1):
              nodes_visited.append(i)
              if (signal.is_set() == False): 
                signal.set()
    else:
      at_end = True
      print("Robot " + str(index) + ": " + str(robot_memory - memory_left))
      # print(nodes_mapped)
      # print("Not enough memory left, done with path at node " + str(current_node))
      # print("Available memory left: " + str(memory_left))
      print("Robot " + str(index) + "'s path: " + str(nodes_visited))
  signal.clear()
  # mark which nodes are mapped after returning to base
  for i in nodes_visited:
    nodes_mapped[i] = 1
  if (not isDone()):
    doneSignal.set()

robot_handler.run_next_robot(uniform_alg,signal,doneSignal)