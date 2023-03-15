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
import problem6_greedy, problem6_jmst, problem6_uniform, problem6_weighted, problem6_speed, problem6_barter
import matplotlib.pyplot as plt
import params
import random, math

greedy_trials = []
jmst_trials = []
uniform_trials = []
weighted_trials = []
speed_trials = []
barter_trials = []

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
barter_rounds = [[]]
x_axis = []

# sum of node memory is a certain ratio of the amount of memory of all the robots, 50-200%
graph_type = 1 # 0 for lattice, 1 for fully connected, 2 for tree
num_trials = 30

# runs robot handler and collects data along the way
def run_robots(a, b):
    if (a > 0 and b == 0):
        greedy_rounds.append([])
        greedy_usage.append([])
        jmst_rounds.append([])
        jmst_usage.append([])
        uniform_rounds.append([])
        uniform_usage.append([])
        weighted_rounds.append([])
        weighted_usage.append([])
        speed_rounds.append([])
        barter_rounds.append([])

    # for i in range(num_trials):
        # running robots returns num rounds and memory usage
    # print("trial #" + str(b))
    greedy_trials.append(robot_handler.run_all_robots(problem6_greedy.greedy_alg))
    jmst_trials.append(robot_handler.run_all_robots(problem6_jmst.jmst_alg))
    speed_trials.append(robot_handler.run_all_robots(problem6_speed.speedy_alg))
    barter_trials.append(robot_handler.run_all_robots(problem6_barter.barter_alg))

    uniform_current = []
    weighted_current = []
    uniform_sum = 0
    weighted_sum = 0
    uniform_avg_usage = numpy.zeros(params.num_nodes)
    weighted_avg_usage = numpy.zeros(params.num_nodes)
    uniform_new = [] 
    weighted_new = []
    for i in range(num_trials):
        uniform_current.append(robot_handler.run_all_robots(problem6_uniform.uniform_alg))
        weighted_current.append(robot_handler.run_all_robots(problem6_weighted.weighted_alg))
    for j in range(num_trials):
        # print(len(uniform_current[j][1]))
        uniform_sum += uniform_current[j][0]
        weighted_sum += weighted_current[j][0]
        # for k in range(len(uniform_current[j][1])):
        #     # print(str(j) + " " + str(k))
        #     uniform_avg_usage[k] += uniform_current[j][1][k]
        #     weighted_avg_usage[k] += weighted_current[j][1][k]
    uniform_avg = uniform_sum/num_trials
    weighted_avg = weighted_sum/num_trials
    # print(str(uniform_avg) + " " + str(weighted_avg))
    uniform_new.append(uniform_avg)
    # uniform_new.append(uniform_avg_usage)
    weighted_new.append(weighted_avg)
    # weighted_new.append(weighted_avg_usage)
    uniform_trials.append(uniform_new)
    weighted_trials.append(weighted_new)

# generates graphs with data collected
def analyze_data():
    print("building graphs (not those kind)")
    fig, rounds= plt.subplots()
    rounds.scatter(numpy.array(x_axis)+0.05, greedy_rounds, label='greedy')
    rounds.scatter(numpy.array(x_axis)+0.15, uniform_rounds, label='uniform random')
    rounds.scatter(numpy.array(x_axis)-0.05, weighted_rounds, label='weighted random')
    rounds.scatter(numpy.array(x_axis)-0.15, jmst_rounds, label='j-mst')
    rounds.scatter(numpy.array(x_axis)-0.10, barter_rounds, label='barter')
    rounds.scatter(numpy.array(x_axis), speed_rounds, label='speed')
    rounds.set_xlabel('node memory (pct of robot)')
    rounds.set_ylabel('rounds to completion')
    rounds.legend()
    rounds.set_xticks([0.5,1,1.5,2])
    rounds.grid(axis = 'y')

    plt.show()

def main():
    for a in range(4):
        for b in range(num_trials):
            print("trial #" + str(b))
            params.node_mem_pct = (a+1)*0.5
            robot_handler.generate_graph(graph_type, params.node_mem_pct)
            params.seed += 1
            run_robots(a, b)

        print("processing data...")
        for i in range(num_trials):
            greedy_rounds[a].append(greedy_trials[(a*num_trials)+i][0])
            greedy_usage[a].extend(greedy_trials[(a*num_trials)+i][1])
            speed_rounds[a].append(speed_trials[(a*num_trials)+i][0])
            jmst_rounds[a].append(jmst_trials[(a*num_trials)+i][0])
            jmst_usage[a].extend(jmst_trials[(a*num_trials)+i][1])
            uniform_rounds[a].append(uniform_trials[(a*num_trials)+i][0])
            # uniform_usage[a].extend(uniform_trials[(a*num_trials)+i][1])
            weighted_rounds[a].append(weighted_trials[(a*num_trials)+i][0])
            # weighted_usage[a].extend(weighted_trials[(a*num_trials)+i][1])
            barter_rounds[a].append(barter_trials[(a*num_trials)+i][0])
            x_axis.append(numpy.array(params.node_mem_pct))
        print("done with runs")
        params.seed = 204
    analyze_data()

if __name__ == "__main__":
    main()