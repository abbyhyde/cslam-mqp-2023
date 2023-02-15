'''
do greedy, see if any trades can be made with earlier routes to create better paths


'''


import random, numpy, math, time, params, robot_handler
import sample_adj_grids_and_memories as sample

def isTravelable(path):
  nodes_visit = path
  nodes_reachable = []
  nodes_reachable.append(nodes_visit[0])
  for curr_node in nodes_reachable:
    for test_node in nodes_visit:
      if adj_grid[curr_node][test_node] == 1 and test_node not in nodes_reachable:
        nodes_reachable.append(test_node)
  if len(nodes_reachable) == len(nodes_visit):
    return True
  return False

def calculate_memory(path):
  total = 0
  for node in path:
    total += node_memory[node]
  return total

def index_of_greatest_mem(possible_nodes):
  highest_cost = node_memory[possible_nodes[0]]
  next_node = possible_nodes[0]
  for i in possible_nodes: #pick nodes greedy
    if(highest_cost < node_memory[i]):
      highest_cost = node_memory[i]
      next_node = i
  return next_node

      

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
  runs = []
  mem_used = []
  while(isDone()):
    at_end = False
    expand = False
    highest_cost = 0
    memory_left = robot_memory
    next_node = 0
    nodes_visited = []
    possible_next_nodes = []
    nodes_visited.append(next_node)
    if(robot == 0 ):
      memory_left = memory_left - node_memory[0]
      nodes_mapped[0] = 1
      
    #consider all visitable nodes except zero
    for j in range(0,nodes):
      if nodes_mapped[j] == 1:
        for i in range(1,nodes):
          if((adj_grid[j][i] == 1)and (j != i) and i not in possible_next_nodes):
            # add number to array
            possible_next_nodes.append(i)
    
    # if robot has enough memory to go to the selected node and robot has not visited it already
    while(len(possible_next_nodes) > 0 and memory_left > 0):
      next_node = index_of_greatest_mem(possible_next_nodes)
      if(node_memory[next_node] <= memory_left):
        if(nodes_mapped[next_node] != 1):#node not already claimed
          nodes_visited.append(next_node)
          memory_left -= node_memory[next_node]
          for i in range(1,nodes):
            if nodes_mapped[i] != 1 and (adj_grid[next_node][i] == 1) and (next_node != i) and i not in possible_next_nodes and i not in nodes_visited:
              possible_next_nodes.append(i)
        else:#try a trade if node is claimed
          runIndex = -1
          for i in range(0,robot): # finds the run where next_node is from
            if next_node in runs[i]:
              runIndex = i
              break
          trial_run = []
          known_nodes = []
          for n in runs[runIndex]:#creating the run without next_node
            if n != next_node:
              trial_run.append(n)
              known_nodes.append(n)
          for i in range(0,runIndex):
            known_nodes.extend(runs[i])# all known nodes except the trade
          done = False
          new_nodes = []
          while(not done):
            new_selection = -1
            new_mem = -1
            for n in known_nodes:
              for i in range(1,nodes):
                if((adj_grid[n][i] == 1)and (n != i) and nodes_mapped[i] != 1 and i not in nodes_visited and \
                  calculate_memory(trial_run) + node_memory[i] <= robot_memory and node_memory[i] > new_mem):
                  new_selection = i
                  new_mem = node_memory[i]
            if new_selection > 0:
              trial_run.append(new_selection) #build out the alternitive
              known_nodes.append(new_selection)
              new_nodes.append(new_selection)
              for i in range(1,nodes):
                if nodes_mapped[i] != 1 and (adj_grid[new_selection][i] == 1) and (new_selection != i) and i not in possible_next_nodes and i not in nodes_visited and i not in known_nodes:
                  possible_next_nodes.append(i)
            else:
              done = True
          if(calculate_memory(trial_run) > calculate_memory(runs[runIndex]) and isTravelable(known_nodes)):
            runs[runIndex] = trial_run
            nodes_visited.append(next_node)
            memory_left -= node_memory[next_node]
            for n in new_nodes:
              nodes_mapped[n] = 1
              for i in range(1,nodes):
                if nodes_mapped[i] != 1 and (adj_grid[n][i] == 1) and (n != i) and i not in possible_next_nodes and i not in nodes_visited:
                  possible_next_nodes.append(i)        
      possible_next_nodes.remove(next_node)
        
        
    
    print("robot-" + str(robot)+ " " + str(robot_memory - memory_left)+ " " + str((time.monotonic_ns()-start)/1000000))
    #print(nodes_mapped)
    # print("Not enough memory left, done with path at node " + str(current_node))
    # print("Available memory left: " + str(memory_left))
    # print(adj_grid[current_node])
    print("Path: " + str(nodes_visited))
    runs.append(nodes_visited)
    # mark which nodes are mapped after returning to base
    robot += 1
    mem_used.append(robot_memory - memory_left)
    for i in nodes_visited:
      nodes_mapped[i] = 1
  rounds.append((numpy.full(len(mem_used), 10) - mem_used).tolist())

for i in range(len(rounds)):
  print(rounds[i])