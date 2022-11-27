import random, numpy, math, time
import params

def calculateNodeWeight(node_weights, index):
  total = 0
  for node_weight in node_weights:
    total += node_weight
  return node_weights[index]/total

#settings params from params.py
nodes = params.nodes
robot_memory = params.memory
robots = params.robots
random.seed(params.seed)
for i in range(0,20): # the number of times we're running the experiment
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
  #print(node_memory)

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


  # weighted probabilistic algorithm

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
    at_end = False
    expand = False
    highest_cost = 0
    memory_left = robot_memory
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
      """
      if there are possible next nodes
        while there are possible next nodes
          randomly pick one (choice through end of while loop)
          if node memory is not the highest cost
            make it the next node
          else
            remove from possible next nodes list
        here if ran out of nodes
        pick one from visited nodes and make it the next node
      """
      # if robot has enough memory to go to the selected node and robot has not visited it already
      size = len(possible_next_nodes)
      if (size > 0):
        while(size > 0):
          if (size == 1):
            next_node = possible_next_nodes[0]
            # print("only size 1, adding " + str(next_node))
            break
          # pick next node
          choice = random.random() - calculateNodeWeight(node_memory, possible_next_nodes[0]) # randomize how much weight we'll go through in order
          next_node_index = 0
          next_node = possible_next_nodes[next_node_index]
          while choice >= 0 and next_node_index < len(possible_next_nodes)-1: # while there's still stuff to go through
            next_node_index = 1 + next_node_index
            next_node = possible_next_nodes[next_node_index]
            choice -= calculateNodeWeight(node_memory, next_node) # remove the next amount of stuff to go through
          if(highest_cost < node_memory[next_node] and nodes_mapped[next_node] != 1):
            highest_cost = node_memory[next_node]
            break
          else:
            # print("removing node " + str(next_node))
            # print(possible_next_nodes)
            possible_next_nodes.remove(next_node)
            size = len(possible_next_nodes)
        nodes_visited.append(next_node)
        nodes_mapped[next_node] = 1
        memory_left -= node_memory[next_node]
        highest_cost = 0
          #print("Next node: " + str(possible_next_nodes[next_node_index]) + ", has cost of " + str(node_memory[possible_next_nodes[next_node_index]]))
          #print("Available memory left: " + str(memory_left))
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
        # print("Available
        # memory left: " + str(memory_left))
        # print(adj_grid[current_node])
        print("Path: " + str(nodes_visited))
        input()