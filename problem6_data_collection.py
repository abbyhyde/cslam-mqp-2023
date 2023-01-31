import robot_handler, problem6_greedy, problem6_jmst, problem6_uniform, problem6_weighted, matplotlib.pyplot as plt, params, numpy, problem6_speed

greedy_trials = [[],[],[]]
jmst_trials = [[],[],[]]
uniform_trials = [[],[],[]]
weighted_trials = [[],[],[]]
speed_trials = [[],[],[]]

amount_robots = [2,5,10]

def main():
    for index in range(len(amount_robots)):
        params.robots = amount_robots[index]
        for i in range(30):
            robot_handler.generate()
            greedy_trials[index].append(robot_handler.run_all_robots(problem6_greedy.greedy_alg))
            jmst_trials[index].append(robot_handler.run_all_robots(problem6_jmst.jmst_alg))
            uniform_trials[index].append(robot_handler.run_all_robots(problem6_uniform.uniform_alg))
            weighted_trials[index].append(robot_handler.run_all_robots(problem6_weighted.weighted_alg))
            speed_trials[index].append(robot_handler.run_all_robots(problem6_speed.speedy_alg))
    greedy_rounds = [[],[],[]]
    greedy_usage = [[],[],[]]
    jmst_rounds = [[],[],[]]
    jmst_usage = [[],[],[]]
    uniform_rounds = [[],[],[]]
    uniform_usage = [[],[],[]]
    weighted_rounds = [[],[],[]]
    weighted_usage = [[],[],[]]
    speed_rounds = [[],[],[]]
    x_axis = [[],[],[]]
    for j in range(len(amount_robots)):
        for i in range(30):
            greedy_rounds[j].append(greedy_trials[j][i][0])
            speed_rounds[j].append(speed_trials[j][i][0])
            greedy_usage[j].extend(greedy_trials[j][i][1])
            jmst_rounds[j].append(jmst_trials[j][i][0])
            jmst_usage[j].extend(jmst_trials[j][i][1])
            uniform_rounds[j].append(uniform_trials[j][i][0])
            uniform_usage[j].extend(uniform_trials[j][i][1])
            weighted_rounds[j].append(weighted_trials[j][i][0])
            weighted_usage[j].extend(weighted_trials[j][i][1])
            x_axis[j].append(numpy.array(amount_robots[j]),)
            
    fig, rounds= plt.subplots()
    rounds.scatter(numpy.array(x_axis)+0.1, greedy_rounds, label='greedy')
    rounds.scatter(numpy.array(x_axis)+0.3, uniform_rounds, label='uniform random')
    rounds.scatter(numpy.array(x_axis)-0.1, weighted_rounds, label='weighted random')
    rounds.scatter(numpy.array(x_axis)-0.3, jmst_rounds, label='j-mst')
    rounds.scatter(numpy.array(x_axis), speed_rounds, label='speed')
    rounds.set_xlabel('number of robots')
    rounds.set_ylabel('rounds to completion')
    rounds.legend()
    rounds.set_xticks([2,5,10])
    rounds.grid(axis = 'y')
    for i in range(len(amount_robots)):
        print(str(amount_robots[i]) + " robots. greedy, mean:" + str(numpy.mean(greedy_rounds[i])))
        print(str(amount_robots[i]) + " robots. jmst, mean:" + str(numpy.mean(jmst_rounds[i])))
        print(str(amount_robots[i]) + " robots. uniform, mean:" + str(numpy.mean(uniform_rounds[i])))
        print(str(amount_robots[i]) + " robots. weighted, mean:" + str(numpy.mean(weighted_rounds[i])))

    plt.show()

if __name__ == "__main__":
    main()