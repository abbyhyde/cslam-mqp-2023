import random, math, params, robot_handler, enum
import problem6_variedgraphcollection as varied_graphs
import igraph as ig
import matplotlib.pyplot as plt

# script to just display the graph, not any robot's path through it

# Construct the graph from the adj matrix
n_vertices = params.num_nodes
edges = []
# robot_handler.generate()
robot_handler.generate_graph(0,0.5)
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
    vertex_label_size=7.0
)
# for each edge between the nodes it mapped, make it a different color

plt.show()

# Save the graph as an image file
fig.savefig('graph.png')

# Export and import a graph as a GML file.
g.save("social_network.gml")
g = ig.load("social_network.gml")