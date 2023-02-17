import random, numpy, math, time, params, robot_handler
import sample_adj_grids_and_memories as sample

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

rounds = []
for k in range(30):
  nodes = len(sample.node_memories[k])
  robot_memory = params.memory
  robots = params.robots
  nodes_mapped = numpy.empty(nodes)

  adj_grid = sample.adj_grids[k]
  node_memory = sample.node_memories[k]
  print("run:" +str(k))
  print(nodes_mapped) #this is needed for some reason?

  # while not at the end or no more possible options
  start = time.monotonic_ns()

  def isDone():
      for i in nodes_mapped:
          if(i != 1):
              return True
      return False

  robot = 0

  mem_used = []
  while(isDone()):
    # mapping the path
    at_end = False
    expand = False
    highest_cost = 0
    memory_left = robot_memory
    if(robot == 0 and nodes_mapped[0] != 1):
      memory_left = memory_left - node_memory[0]
      nodes_mapped[0] = 1
    next_node = 0
    current_node = 0
    nodes_visited = []
    possible_next_nodes = []
    pick = 0
    nodes_visited.append(current_node)
    # while not at the end or no more possible options
    start = time.monotonic_ns()
    while (not at_end):
      # print("Robot " + str(index) + " starting loop " + str(at_end))
      possible_next_nodes = []
      # can only consider a node if there's an edge and hasn't been visited yet and has a smaller memory cost
      for j in nodes_visited:
        for i in range(0,nodes):
          if((adj_grid[j][i] == 1) and (i not in nodes_visited) and (j != i) and node_memory[i] <= memory_left and nodes_mapped[i] != 1):
            # add number to array
            possible_next_nodes.append(i)
      
      # if robot has enough memory to go to the selected node and robot has not visited it already
      if (len(possible_next_nodes) > 0):
        # print("Robot " + str(index) + " where we want")
        next_node = possible_next_nodes[math.floor(random.random()*len(possible_next_nodes))]
        nodes_visited.append(next_node)
        memory_left -= node_memory[next_node]
        highest_cost = 0
      elif(not expand):
        expand = True
        for i in range(1, len(nodes_mapped)):
            if (i not in nodes_visited and nodes_mapped[i] == 1):
                nodes_visited.append(i)
      else:
        at_end = True
        print("Robot " + str(robot) + ": " + str(robot_memory - memory_left))
        # print(nodes_mapped)
        # print("Not enough memory left, done with path at node " + str(current_node))
        # print("Available memory left: " + str(memory_left))
        print("Robot " + str(robot) + "'s path: " + str(nodes_visited))
        # mark which nodes are mapped after returning to base
        mem_used.append(robot_memory - memory_left)
        robot += 1
        for i in nodes_visited:
            nodes_mapped[i] = 1
  rounds.extend((numpy.full(len(mem_used), 10) - mem_used).tolist())

for i in range(len(rounds)):
  print(rounds[i])