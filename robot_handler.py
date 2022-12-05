""" 
Generates graph for each runthrough as well as manages the thread for each robot. 
"""
import random, numpy, math, time, logging, threading
import params

#settings params from params.py
nodes = params.nodes
robot_memory = params.memory
robots = params.robots
random.seed(params.seed)

def generate():
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
    return adj_grid, node_memory

def run_next_robot(algorithm,signal,doneSignal):
    threads = []
    # runs all robots
    index = 0
    while (not doneSignal.is_set()):
        if (len(threads) == index):
            print("     Main: create and start robot " + str(index))
            new_thread = threading.Thread(target=algorithm, args=(index,))
            threads.append(new_thread)
            new_thread.start()
            signal.wait(timeout=60.0)
        elif (threads[index].is_alive()):
            continue
        else:
            print("     Main: restart robot " + str(index))
            threads[index] = threading.Thread(target=algorithm, args=(index,))
            new_thread = threads[index]
            new_thread.start()
            signal.wait(timeout=60.0)
        index = (index+1) % robots

    for index, thread in enumerate(threads):
        thread.join()
        print("     Main: done with robot " + str(index))
