import gurobipy as gp, random, numpy, math, time, params, robot_handler
from gurobipy import GRB
from itertools import combinations
import sample_adj_grids_and_memories as sample

mem_used = []
paths = []
for i in range(100):
#settings params from params.py
  nodes = params.num_nodes
  robot_memory = params.memory
  robots = params.robots
  

  adj_grid = sample.adj_grids[math.ceil(i/30)%30]
  node_memory = sample.node_memories[i%30]
        
  #Gurobi code below

  #converting the adj grid to a dict
  edge_dict = {(v1, v2): adj_grid[v1][v2] for v1, v2 in combinations(range(0, nodes), 2)} # add all the edges in one direction

  weight_dict = {v: node_memory[v] for v in range(0, nodes)}

  vertex_labels = {v: v for v in range(0,nodes)}

  #create the model and add the edges
  m = gp.Model()
  vertexes = m.addVars(vertex_labels.keys(), vtype=GRB.BINARY, name='v')
  edges = m.addVars(edge_dict.keys(), vtype=GRB.BINARY, name='e')
  for i, j in edges.keys(): # reverses the edges of the direction
      edges[j, i] = edges[i, j]
      
  e_keys = []

  for key in edge_dict.keys():
    e_keys.append(key)


  m.setObjective(vertexes.prod(weight_dict), GRB.MAXIMIZE)

  # Constraints:
  m.addConstr(vertexes.prod(weight_dict) <= robot_memory) #ensures we never use more that the memory constraint
  m.addConstr(vertexes[0] == 1) #ensures we move through the entry node at 0
  #m.addConstr(vertexes.sum()-1 ==  gp.quicksum(vertexes[v]*vertexes[u]*edge_dict[(v,u)] for v,u in combinations(vertex_labels.keys(), 2)))
  #^ says that the number of edges is at least the number of verticies-1
  m.addConstrs(vertexes[v]*edges.sum(v,'*') == edges.sum(v,'*') for v in vertex_labels.keys()) # only used vertexes have edges
  m.addConstrs(vertexes[v]*edges.sum(v,'*') >= vertexes[v]      for v in vertex_labels.keys()) # all vertexes have edges


  def sparse_connection_elim(model, where):
    if where == GRB.Callback.MIPSOL:
      vals = model.cbGetSolution(model.getVars())
      v = vals[0:len(vertex_labels.keys())]
      e = vals[len(vertex_labels.keys()):]
      path = []
      used_edges = []
      for i in vertex_labels.keys():
        if v[i] > 0.5:
          path.append(i)
      for i in range(len(e_keys)):
        if e[i] > 0.5:
          used_edges.append(e_keys[i])
      if(detectCycles(used_edges)):
        print(path)
        print(used_edges)
        model.cbLazy(gp.quicksum(edges[i, j] for i, j in combinations(path, 2))<= len(path)-1)
          
  def detectCycles(edges):
    edges_left = edges.copy()
    nodes_visited = []
    queue = []
    queue.append(edges[0][0])
    while(len(queue) > 0 and len(edges_left) > 0): # performs bfs across the solution trying to find and edges back to previously seen nodes
      curr_node = queue.pop()
      nodes_visited.append(curr_node)
      edges_to_remove = []
      for edge in edges_left:
        if edge[0] == curr_node:
          if nodes_visited.count(edge[1]) > 0 or queue.count(edge[1]) > 0:
            return True
          else:
            queue.append(edge[1])
            edges_to_remove.append((curr_node,edge[1]))
        elif edge[1] == curr_node:
          if nodes_visited.count(edge[0]) > 0 or queue.count(edge[0]) > 0:
            return True
          else:
            queue.append(edge[0])
            edges_to_remove.append((edge[0],curr_node))
      for edge in edges_to_remove:
        edges_left.remove(edge)
    if len(queue) == 0:
      return True
    return False
      
    
          
  m.Params.lazyConstraints = 1
  m.optimize(sparse_connection_elim)

  v_sol = m.getAttr('x', vertexes)
  e_sol = m.getAttr('x', edges)
  path_sol = []
  used_edges_sol = []
  total = 0
  for i in vertex_labels.keys():
    if v_sol[i] > 0.5:
      total += node_memory[i]
      path_sol.append(i)
  for i in e_keys:
    if e_sol[i] > 0.5:
      used_edges_sol.append(i)
      
  print('Memory Used: '+ str(total))
  print('Nodes visited: ' + str(path_sol))
  print('Edges Traversed: ' + str(used_edges_sol))
  mem_used.append(total)
  paths.append(path_sol)

  m.reset(0)

for i in range(len(mem_used)):
  print(mem_used[i])
  #print(paths[i])