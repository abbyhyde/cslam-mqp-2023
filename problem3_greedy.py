import random, numpy, math, time, threading
import params, robot_handler

nodes = params.nodes
robot_memory = params.memory
robots = params.robots
random.seed(params.seed)

nodes_mapped = numpy.empty(nodes)
adj_grid, node_memory = robot_handler.generate()
signal = threading.Event()
doneSignal = threading.Event()

def isDone():
  for i in nodes_mapped:
      if(i != 1):
          return True
  return False

def greedy_alg(index):
  # greedy algorithm
  # adds the highest possible cost to explore the longest as possible

  # mapping the path
  # make that the next node
  start = time.monotonic_ns()
  at_end = False
  expand = False
  highest_cost = 0
  memory_left = params.memory
  next_node = 0
  nodes_visited = []
  possible_next_nodes = []
  nodes_visited.append(next_node)
  while (not at_end):
    # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
    possible_next_nodes = []
    # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
    for j in nodes_visited:
      for i in range(0,nodes):
        if((adj_grid[j][i] == 1) and (i not in nodes_visited) and (j != i) and node_memory[i] <= memory_left and nodes_mapped[i] != 1):
          # add number to array
          possible_next_nodes.append(i)
    
    # if robot has enough memory to go to the selected node and robot has not visited it already
    if (len(possible_next_nodes) > 0):
      highest_cost = node_memory[possible_next_nodes[0]]
      next_node = possible_next_nodes[0]
      for i in possible_next_nodes:
        if(highest_cost < node_memory[i]):
          highest_cost = node_memory[i]
          next_node = i
      nodes_visited.append(next_node)
      nodes_mapped[next_node] = 1
      memory_left -= node_memory[next_node]
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
        print("robot-" + str(index)+ " " + str(robot_memory - memory_left)+ " " + str((time.monotonic_ns()-start)/1000000))
        #print(nodes_mapped)
        # print("Not enough memory left, done with path at node " + str(current_node))
        # print("Available memory left: " + str(memory_left))
        # print(adj_grid[current_node])
        print("Path: " + str(nodes_visited))
  signal.clear()
  # mark which nodes are mapped after returning to base
  for i in nodes_visited:
    nodes_mapped[i] = 1
  if (not isDone()):
    doneSignal.set()

robot_handler.run_next_robot(greedy_alg,signal,doneSignal)