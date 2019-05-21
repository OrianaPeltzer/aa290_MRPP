# Here is a setup and some associated functions
# Setup 2 is a 10x10 dense grid factory

import matplotlib.pyplot as plt
from matplotlib.contour import ContourSet
import matplotlib.cm as cm
import numpy as np
from Graph import graph, create_graph_factory1, create_dense_graph
from create_setup import factory, obstacle, machine, wall, robot
from IPython import embed