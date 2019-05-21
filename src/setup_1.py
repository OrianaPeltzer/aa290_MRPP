# Here is a setup and some associated functions
# Setup 1 is a small factory.

import matplotlib.pyplot as plt
from matplotlib.contour import ContourSet
import matplotlib.cm as cm
import numpy as np
from Graph import graph, create_graph_factory1, create_dense_graph
from IPython import embed
from create_setup import factory, obstacle, machine, wall, robot


if __name__ == "__main__":

    # Create graph
    mygraph = create_dense_graph()

    # Set starting and goal nodes
    sources = [((1, 1), 1, 0), ((1, 5), 1, 0), ((3, 3), 1, 0), ((8, 2), 1, 0), ((3, 9), 1, 0),((7, 6), 1, 0),((9, 5), 1, 0),((8, 4), 1, 0),((8, 5), 1, 0),((8, 6), 1, 0)]
    sinks = [((1, 9), 1, 19), ((3, 5), 1, 19), ((5, 8), 1, 19), ((2, 5), 1, 19), ((8, 8), 1, 19),((5, 1), 1, 19),((6, 6), 1, 19),((1, 8), 1, 19),((3, 1), 1, 19),((4, 1), 1, 19)]

    # Create factory object with sinks as machines
    Factory = factory(graph=mygraph,machine_sinks=sinks)

    # Visualize factory floor
    Factory.plot_floor(graph=False)

    # Create and solve flow problem
    #Factory.mrpp_graph.create_flow_problem(sources=sources,sinks=sinks)
    Factory.mrpp_graph.create_perturbed_flow_problem(sources=sources, sinks=sinks)

    # Plot flow solution
    # Factory.plot_flow_solution()

    # Extract solution
    path = Factory.mrpp_graph.solution_to_labeled_path()
    print(path)

