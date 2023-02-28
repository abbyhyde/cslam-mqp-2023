"""
Runs data collection for each algorithm for three types of graphs in problem 6.
The three types of graphs are lattice, fully connected, and tree. 
Algorithm data labels, % node memory (x) vs round to completion (y) by graph type
Graph shape not yet tested cause ubuntu is stupid with igraph apparently
"""
# tasks:
# test running robots / collecting data
import numpy
import robot_handler
import problem6_greedy, problem6_jmst, problem6_uniform, problem6_weighted, problem6_speed
import matplotlib.pyplot as plt
import params
import random, math

greedy_trials = []
jmst_trials = []
uniform_trials = []
weighted_trials = []
speed_trials = []

# stores the data for each run? idk
greedy_rounds = [[]]
greedy_usage = [[]]
jmst_rounds = [[]]
jmst_usage = [[]]
uniform_rounds = [[]]
uniform_usage = [[]]
weighted_rounds = [[]]
weighted_usage = [[]]
speed_rounds = [[]]
x_axis = []

# sum of node memory is a certain ratio of the amount of memory of all the robots, 50-200%
graph_type = 2 # 0 for lattice, 1 for fully connected, 2 for tree
num_trials = 30

# runs robot handler and collects data along the way
def run_robots(a):
    if (a > 0):
        greedy_rounds.append([])
        greedy_usage.append([])
        jmst_rounds.append([])
        jmst_usage.append([])
        uniform_rounds.append([])
        uniform_usage.append([])
        weighted_rounds.append([])
        weighted_usage.append([])
        speed_rounds.append([])

    for i in range(num_trials):
        # running robots returns num rounds and memory usage
        # print("trial #" + str(i))
        greedy_trials.append(robot_handler.run_all_robots(problem6_greedy.greedy_alg))
        jmst_trials.append(robot_handler.run_all_robots(problem6_jmst.jmst_alg))
        uniform_trials.append(robot_handler.run_all_robots(problem6_uniform.uniform_alg))
        weighted_trials.append(robot_handler.run_all_robots(problem6_weighted.weighted_alg))
        speed_trials.append(robot_handler.run_all_robots(problem6_speed.speedy_alg))
    
    print("processing data...")
    for i in range(num_trials):
        greedy_rounds[a].append(greedy_trials[(a*num_trials)+i][0])
        greedy_usage[a].extend(greedy_trials[(a*num_trials)+i][1])
        speed_rounds[a].append(speed_trials[(a*num_trials)+i][0])
        jmst_rounds[a].append(jmst_trials[(a*num_trials)+i][0])
        jmst_usage[a].extend(jmst_trials[(a*num_trials)+i][1])
        uniform_rounds[a].append(uniform_trials[(a*num_trials)+i][0])
        uniform_usage[a].extend(uniform_trials[(a*num_trials)+i][1])
        weighted_rounds[a].append(weighted_trials[(a*num_trials)+i][0])
        weighted_usage[a].extend(weighted_trials[(a*num_trials)+i][1])
        x_axis.append(numpy.array(params.node_mem_pct))
    
    print("done with runs")

# generates graphs with data collected
def analyze_data():
    print("building graphs (not those kind)")
    fig, rounds= plt.subplots()
    rounds.scatter(numpy.array(x_axis)+0.05, greedy_rounds, label='greedy')
    rounds.scatter(numpy.array(x_axis)+0.15, uniform_rounds, label='uniform random')
    rounds.scatter(numpy.array(x_axis)-0.05, weighted_rounds, label='weighted random')
    rounds.scatter(numpy.array(x_axis)-0.15, jmst_rounds, label='j-mst')
    rounds.scatter(numpy.array(x_axis), speed_rounds, label='speed')
    rounds.set_xlabel('node memory (pct of robot)')
    rounds.set_ylabel('rounds to completion')
    rounds.legend()
    rounds.set_xticks([0.5,1,1.5,2])
    rounds.grid(axis = 'y')

    plt.show()

def main():
    for a in range(4):
        params.node_mem_pct = (a+1)*0.5
        robot_handler.generate_graph(graph_type, params.node_mem_pct)
        run_robots(a)
    analyze_data()

if __name__ == "__main__":
    main()