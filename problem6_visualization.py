import random, math, params, robot_handler, enum
import problem6_uniform as uniform
import problem6_speed as speed
import problem6_greedy as greedy
import igraph as ig
import matplotlib.pyplot as plt

# visualizes the graph with each robot's path through it in a separate file
# will be fixed to show the order of each edge explored and other cool stuff

# Construct the graph from the adj matrix
n_vertices = params.num_nodes
edges = []
robot_handler.generate()
robot_handler.run_all_robots(uniform.uniform_alg)
# print(robot_handler.adj_grid)
for i in range(n_vertices):
    for j in range(n_vertices):
        if robot_handler.adj_grid[i][j] == 1 and i > j:
            new_edge = (i, j)
            edges.append(new_edge)
# print(edges)
g = ig.Graph(n_vertices, edges)

# Set attributes for the graph, nodes, and edges
g["title"] = "C-SLAM Visualization"
names = ["entry node 0"]
for i in range(n_vertices-1):
    names.append("node" + str(i+1))
g.vs["name"] = names
# assigns status to each node but doesn't really do anything with the info atm
status = []
for j in range(len(robot_handler.nodes)):
    if robot_handler.nodes[j] == robot_handler.Robot_State.MAPPED:
        status.append("M")
    elif robot_handler.nodes[j] == robot_handler.Robot_State.CLAIMED:
        status.append("C")
    elif robot_handler.nodes[j] == robot_handler.Robot_State.NOT_CLAIMED:
        status.append("NC")
    else:
        status.append("NE")
g.vs["status"] = status

# grab results of algorithm after it's done somehow -> path field
# for each robot make a plot
for r in range(params.robots): 
    # generate the edges of the robot's path
    curr_robot = robot_handler.robots[r]
    # print("---------")
    # print(r)
    print(curr_robot.path)
    path_edges = []
    for i in range(len(curr_robot.path)):
        new_edge = (curr_robot.path[i][0], curr_robot.path[i][1])
        path_edges.append(new_edge)
        new_edge = (curr_robot.path[i][1], curr_robot.path[i][0])
        path_edges.append(new_edge)
    # print(path_edges)

    # Plot in matplotlib
    # Note that attributes can be set globally (e.g. vertex_size), or set individually using arrays (e.g. vertex_color)
    fig, ax = plt.subplots(figsize=(5,5))
    ig.plot(
        g,
        target=ax,
        layout="circle", # print nodes in a circular layout
        vertex_size=0.1,
        # vertex_color=ig.RainbowPalette(4).get(),
        vertex_color="steelblue",
        vertex_frame_width=4.0,
        vertex_frame_color="white",
        vertex_label=g.vs["name"],
        vertex_label_size=7.0,
        # edge_color=["#7142cf" if married else "#AAA" for married in g.es["married"]]
        edge_color=["#7142cf" if e in path_edges else "#AAA" for e in edges]
    )
    # for each edge between the nodes it mapped, make it a different color

    plt.show()

    # Save the graph as an image file
    fig.savefig(str(r) + '_problem6visualization.png')

    # Export and import a graph as a GML file.
    g.save("social_network.gml")
    g = ig.load("social_network.gml")