# Here is where the graph object is created.
import matplotlib.pyplot as plt
from gurobipy import *
import numpy as np
import copy

class graph():
    """ This unidirected graph has vertexes and edges. Each edge has a cost and a capacity.
    self.g = { vertex: {vertex2: [(cost1,capacity1),(cost2,capacity2)]}, nextvertex: ... }
    a vertex is a tuple of integers (x,y)"""
    def __init__(self):
        self.g = {}

    def add_edges_bidirectional(self, edges):
        """ edges is a list of tuples (vertex1, vertex2, cost, capacity)"""
        for edge in edges:

            #Add the first edge as it is
            try:
                #There is already an edge between both nodes
                self.g[edge[0]][edge[1]] += [(edge[2],edge[3])]
            except:
                try:
                    #There is already an edge that starts at node1 but not one that goes to node2
                    self.g[edge[0]][edge[1]] = [(edge[2], edge[3])]
                except:
                    #This is the first edge that starts at node1
                    self.g[edge[0]] = {edge[1]: [(edge[2], edge[3])]}

            #Add the reverse edge
            try:
                #There is already an edge between both nodes
                self.g[edge[1]][edge[0]] += [(edge[2],edge[3])]
            except:
                try:
                    #There is already an edge that starts at node1 but not one that goes to node2
                    self.g[edge[1]][edge[0]] = [(edge[2], edge[3])]
                except:
                    #This is the first edge that starts at node1
                    self.g[edge[1]] = {edge[0]: [(edge[2], edge[3])]}

    def add_edges_implicitly_bidirectional(self, edges):
        """ edges is a list of tuples (vertex1, vertex2, cost, capacity)"""
        for edge in edges:

            #Add the first edge as it is
            try:
                #There is already an edge between both nodes
                self.g[edge[0]][edge[1]] += [(edge[2],edge[3])]
            except:
                try:
                    #There is already an edge that starts at node1 but not one that goes to node2
                    self.g[edge[0]][edge[1]] = [(edge[2], edge[3])]
                except:
                    #This is the first edge that starts at node1
                    self.g[edge[0]] = {edge[1]: [(edge[2], edge[3])]}

    def define_vertices(self,vertices):
        self.vertices = vertices

    def plot_in_factory(self):
        nodes = self.g.keys()
        for k,node in enumerate(nodes):
            x = node[0] + 0.5
            y = node[1] + 0.5
            plt.scatter(x, y, s=20, marker=(5, 0, 180), c='b')
            for node2 in self.g[node].keys():
                xp = node2[0] + 0.5
                yp = node2[1] + 0.5
                line_to_plot = [(x,xp),(y,yp),'g']
                plt.plot(*line_to_plot)

    def create_flow_problem(self,sources=[((1,1),2,0)],sinks=[((3,6),1,20),((8,5),1,20)],time_horizon=20):
        """sources/sinks is a list of tuples (vertex,number of robots coming/going,time_index)
        time_horizon should be equal to the max time in the sinks list. Let's not spend time looking for
        the max and add it as input to the function directly"""

        #Initialize time to 0
        t = 0

        #Get number of robots. We only go through all the robots when we set the constraints, not when we create the model.
        num_robots = 0
        for k in range(len(sources)):
            num_robots += sources[k][1]

        #Create the gurobi model
        self.m = Model("Factory")

        # For the moment here we consider that the capacity (in terms of number of robots) of each vertex is 5.
        vertex_robot_capacity = 5

        # Initialize an empty dictionary
        self.flow_graph = {}




        # ------------------------------- CHANNEL VARIABLES AND CONSTRAINTS ------------------------------------------ #


        self.channel_dict = {} #Keeps track of which channel is which set of variables
        self.channel_vars = [] #Stores the integer variables
        channel_counter = 0 #Used to create these two

        # We will want to loop through all edges, so for this we need to loop through all keys in the dictionary. Here we use the implicitly bidirectional structure
        nodes_to_go_through = self.g.keys()

        # Let's start creating the graph
        for node1 in nodes_to_go_through:
            node2s = self.g[node1].keys()
            for node2 in node2s:
                possibilities = self.g[node1][node2]

                #We abandon the possibility of reasoning with equivalent costs since there may be a different amount of time required for each channel
                #possibilities = np.array(possibilities)
                #equivalent_capacity = possibilities.sum(0)[1]
                #equivalent_cost = np.sum(np.dot(possibilities[::,0],possibilities[::,1]))/equivalent_capacity

                for k,possibility in enumerate(possibilities):
                    # A possibility is a tuple (cost, capacity) for a channel

                    # We now have a channel at time t.
                    # Let's create binary variables for it

                    t_edge = possibility[0] # Let's say time = cost here
                    capacity = possibility[1] # max number of robots we want in the channel (equivalent to max number we want on the edge)

                    # Store these in our channel dictionary under the format x_node1_node2_t0, x_node2_node1_t0, x_node1_node2_t1, x_node2_node1_t1,...
                    # k should always be 0 except when we have two edges in parallel

                    self.channel_dict[(node1, node2,k)] = (copy.deepcopy(channel_counter),t_edge, capacity)


                    self.channel_vars += [self.m.addVars(2,time_horizon+t_edge,lb=0.0,ub=capacity+0.1, vtype=GRB.INTEGER,name="channel"+str(channel_counter))]
                    # The capacity+0.1 is just to make sure not to have problems in case the bound is exclusive
                    channel_counter += 1

                    # Now that we create all variables for our channels, we can add our channel constraints by going through all of them.
                    # A channel constraint is sum(x12k+x21k) for all xs that take value at t (so all k between t-tedge and t) < channel_capacity
                    # We need to watch out for beginning and end constraints: we do not consider edges that start before time 0, but we do
                    # consider edges that end after horizon T.

        #We get out of the loop in order to only have to update once
        self.m.update()

        #Now we go back in the channel loop that we created:
        for channel_key in self.channel_dict.keys():

            #Let's first retrieve everything from
            channel = self.channel_dict[channel_key]
            channel_index = channel[0]
            channel_time = channel[1]
            channel_capacity = channel[2]
            channel_variable_tupledict = self.channel_vars[channel_index]

            for t in range(time_horizon+channel_time): #We end at T + channel time because the last departures start at T
                channel_flow = LinExpr()
                if t < channel_time:
                    for t_2 in range(t):
                        channel_flow += channel_variable_tupledict[0,t_2]
                        channel_flow += channel_variable_tupledict[1,t_2]
                elif t <= time_horizon:
                    for k in range(channel_time):
                        channel_flow += channel_variable_tupledict[0,t-k]
                        channel_flow += channel_variable_tupledict[0,t-k]
                else:
                    for t_2 in range(t-time_horizon):
                        channel_flow += channel_variable_tupledict[0,time_horizon+t_2]
                        channel_flow += channel_variable_tupledict[1,time_horizon+t_2]

                self.m.addConstr(channel_flow <= channel_capacity,name="channel_constraint_"+str(channel_index)+"_t"+str(t))
        self.m.update()

        # ------------------------------------------------------------------------------------------------------------ #


        # --------------------------- STATIONARY EDGE VARIABLES AND CONSTRAINTS -------------------------------------- #

        # Green and blue arrows in the paper
        self.vertex_dict = {} # This dictionary assigns a node to an index
        self.green_vars = []

        for node_index,node_tuple in enumerate(self.vertices):
            node = node_tuple[0]
            node_capacity = node_tuple[1]
            self.vertex_dict[node] = node_index #this is not fundamentally useful but more rigorous

            # Create green variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.m.addVars(time_horizon, lb=0.0, ub=node_capacity+0.1, vtype=GRB.INTEGER,
                           name="green_" + str(node_index))

            # Create blue variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.m.addVars(time_horizon, lb=0.0, ub=node_capacity + 0.1, vtype=GRB.INTEGER,
                           name="blue_" + str(node_index))

        self.m.update()

        # ------------------------------------------------------------------------------------------------------------ #


        # ----------------------------------------- FLOW CONSTRAINTS ------------------------------------------------- #
        for node_tuple in self.vertices:
            node = node_tuple[0]

            # We get the node index in order to be able to access blue and green variables
            node_index = self.vertex_dict[node]

            # We get the indexes of all neighboring channels to access the gadget variables
            list_of_channel_indexes = self.get_neighbor_channel_indexes(node)

            # Let's start with all the constraints at t0 which will include the source term(s)
            departing_at_t0 = LinExpr()
            departing_at_t0 += self.green_vars[node_index][0]
            for ch_idx in list_of_channel_indexes:
                departing_at_t0 += self.channel_vars[ch_idx][0,0]





    def get_neighbor_channel_indexes(self,node):
        # First we go through the original dictionary of edges
        channel_indexes = []

        for channel_tuple in self.channel_dict.keys():
            node1 = channel_tuple[0]
            node2 = channel_tuple[1]
            if node1 == node or node2 == node:
                channel_indexes += [self.channel_dict[channel_tuple][0]]

        return channel_indexes

















def create_graph_factory1():
    my_graph = graph()
    edges = [((1,1),(1,3),3,3),   ((1,3),(3,6),3,3),  ((1,3),(4,3),3,1),  ((1,3),(4,3),2,1),  ((3,6),(13,6),5,4),  ((13,6),(8,5),3,10),
             ((8,5),(4,3),3,3),  ((4,3),(13,6),8,8) ]
    list_of_vertices = [((1,1),4),((1,3),2),((3,6),1),((4,3),3),((13,6),5),((8,5),1)]
    my_graph.add_edges_implicitly_bidirectional(edges)
    my_graph.define_vertices(list_of_vertices)
    return my_graph
