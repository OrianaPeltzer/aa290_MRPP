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
    degree = 1 # Degree of polynomial to fit.
    num_training_samples = 500 # number of samples to generate before fitting
    num_testing_samples = 10 # number of samples to test our model on

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
    #model = make_pipeline(PolynomialFeatures(degree,interaction_only=True), RidgeCV(alphas=np.linspace(0.01, 10, 10)))
    model = make_pipeline(PolynomialFeatures(degree, interaction_only=True), RidgeCV(alphas=[1]))

    # Fit model
    print("Fitting Model")
    result = model.fit(X,Y)
    print("Model Fit")

    # Get score of the model on training data
    score = result.score(X,Y)

    # Get score of the model on test data
    test_score = result.score(X_test,Y_test)

    # # Get convergence plot
    # training_scores = []
    # test_scores = []
    # ks = []
    # for k in range(1,num_training_samples,int(num_training_samples/100)+1):
    #     result = model.fit(X[:k],Y[:k])
    #     training_score = result.score(X[:k],Y[:k])
    #     test_score = result.score(X_test,Y_test)
    #     training_scores += [training_score]
    #     test_scores += [test_score]
    #     ks += [k]
    #
    # plt.close()
    # plt.plot(ks,training_scores)
    # plt.plot(ks,test_scores)
    # plt.legend(["Training", "Testing"])
    # plt.title("Convergence Plot for training dataset")
    # plt.xlabel("Number of training samples")
    # plt.ylabel("Score")
    # plt.show()

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
        if k%100==0:
            print(k, " over ", len(X[0]))
    # -------------------------------------------------------

    # Extract mean
    mn = np.mean(coeffs)

    # Find the important coefficient indexes
    idxs = np.where([np.abs(k-mn) >= 0.001 for k in coeffs])[0]

    # Reduce X using these indexes
    X_reduced = [xb[idxs] for xb in X]

    # Term to substract to Y in order to correspond to X_reduced
    Y_reduced = []
    for k,yb in enumerate(Y):
        Ymm = yb - np.sum([mn*xelt for xelt in X[k]])
        Y_reduced += [Ymm]

    # Now we can attempt to fit second order polynomial model
    embed()

    # Back to original model to get robust solution
    Factory.mrpp_graph.find_robust_solution(coeffs,sources=sources, sinks=sinks)

    # Extract variables
    xR = Factory.mrpp_graph.get_x()

    # Extract expected cost yR
    solution_R, yR = Factory.mrpp_graph.get_solution_cost(num_particles=100)

    embed()


