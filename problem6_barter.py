import params, robot_handler
nodes = robot_handler.nodes
robot_memory = params.memory
robots = params.robots

def barter_alg(adj_grid, node_memory, nodes_to_visit, max_memory, nodes):
    #greedly choose a node 
    #2 possibilities node is clamed by robot a or not
    #if not, everything is fine
    #if it is claimed, have the robot pick new nodes 
    #have robot a choose new nodes or not
    #compare if a' and currpath+ new node is better that a and currpath
    #go through with trade if a'.length <= a.length,  decreases or equals variance in length in paths, a.memoryleft >= a'.memoryleft #possible margin
    #use meta heuristics to weight each decision.
    pass
    

def main():
    robot_handler.generate()
    robot_handler.run_all_robots(barter_alg)

if __name__ == "__main__":
    main()