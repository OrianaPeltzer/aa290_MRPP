# Here is a setup and some associated functions
# Setup 2 is a 10x10 dense grid factory

import matplotlib.pyplot as plt
from matplotlib.contour import ContourSet
import matplotlib.cm as cm
import numpy as np
from Graph import graph, create_graph_factory1, create_dense_graph
from create_setup import factory, obstacle, machine, wall, robot
from IPython import embed
import copy

# Sklearn
from sklearn.linear_model import RidgeCV
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

if __name__ == "__main__":

    #Hyperparameters---
    degree = 1 # Degree of polynomial to fit. This is a linear regression - actually a degree more than 1 is nonesense
    num_training_samples = 1000 # number of samples to generate before fitting
    num_testing_samples = 20 # number of samples to test our model on

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
    Factory.mrpp_graph.create_flow_problem(sources=sources,sinks=sinks)

    # Extract solution x0
    x0 = Factory.mrpp_graph.get_x()

    # Extract expected cost y0
    solution_0,y0 = Factory.mrpp_graph.get_solution_cost(num_particles=100)

    # ----------------- Sample num_samples perturbed problems and get their costs ------------------------
    X = [x0] # Training data
    Y = [y0]
    paths=[solution_0]

    X_test = [] # Testing data
    Y_test = []
    test_paths = []

    for k in range(num_training_samples+num_testing_samples):

        # Sample new point
        Factory.mrpp_graph.create_perturbed_flow_problem(sources=sources, sinks=sinks)
        x = Factory.mrpp_graph.get_x()
        solutionpath, y = Factory.mrpp_graph.get_solution_cost(num_particles=100)

        # Did we already use this point?
        doubled_value = False
        for sample_x in X:
            if sample_x == x:
                doubled_value = True
                continue
        for sample_x in X_test:
            if sample_x == x:
                doubled_value = True
                continue

        # Add the point if not
        if doubled_value == False:
            if k < num_training_samples:
                X += [x]
                Y += [y]
                paths += [solutionpath]
            else:
                X_test += [x]
                Y_test += [y]
                test_paths += [solutionpath]
    # ---------------------------------------------------------------------------------------------------

    # Turn data into numpy arrays
    X = np.array(X)
    Y = np.array(Y)

    # Prepare model
    model = make_pipeline(PolynomialFeatures(degree), RidgeCV(alphas=np.linspace(0.01, 10, 10)))

    # Fit model
    result = model.fit(X,Y)

    # Get score of the model on training data
    score = result.score(X,Y)

    # Get score of the model on test data
    test_score = result.score(X_test,Y_test)

    # Extract all coefficients one by one --------------------

    x = np.array([[0.0 for k in range(len(X[0]))]])
    coeffs = [model.predict(x)]

    for k in range(len(X[0])):
        # add a 1 at the term that you want to extract
        x[0][k] = 1.0
        try:
            x[0][k-1] = 0.0
        except:
            pass
        coeffs += [model.predict(x)]
        embed()
        if k%100==0:
            print(k, " over ", len(X[0]))
    # -------------------------------------------------------

    # Back to original model to get robust solution
    Factory.mrpp_graph.find_robust_solution(coeffs,sources=sources, sinks=sinks)

    # Extract variables
    xR = Factory.mrpp_graph.get_x()

    # Extract expected cost yR
    solution_R, yR = Factory.mrpp_graph.get_solution_cost(num_particles=100)

    embed()


