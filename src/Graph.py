# Here is where the graph object is created.
import matplotlib.pyplot as plt
from gurobipy import *
import numpy as np
import copy
from IPython import embed

class graph():
    """ This unidirected graph has vertexes and edges. Each edge has a cost and a capacity.
    self.g = { vertex: {vertex2: [(cost1,capacity1),(cost2,capacity2)]}, nextvertex: ... }
    a vertex is a tuple of integers (x,y)"""
    def __init__(self):
        self.g = {}

        # For extracting paths later on
        self.occupancy_dict = {}

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

    def create_flow_problem(self,sources=[((1,1),2,0)],sinks=[((3,6),1,19),((8,5),1,19)],time_horizon=20):
        """sources/sinks is a list of tuples (vertex,number of robots coming/going,time_index)
        time_horizon should be equal to the max time in the sinks list. Let's not spend time looking for
        the max and add it as input to the function directly"""

        #Get number of robots. We only go through all the robots when we set the constraints, not when we create the model.
        num_robots = 0
        for k in range(len(sources)):
            num_robots += sources[k][1]

        #Create the gurobi model
        self.m = Model("Factory")


        # Initialize an empty dictionary
        self.flow_graph = {}

        self.time_horizon = time_horizon




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

                    ##Now let's add to the channel dictionary indexed by edge time
                    #try:
                    #    self.channel_by_time_dict[t_edge] += [copy.deepcopy(channel_counter)]
                    #except:
                    #    self.channel_by_time_dict[t_edge] = [copy.deepcopy(channel_counter)]


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

            for t in range(1,time_horizon+channel_time): #We end at T + channel time because the last departures start at T
                channel_flow = LinExpr()
                if t < channel_time:
                    for t_2 in range(t+1):
                        channel_flow += channel_variable_tupledict[0,t_2]
                        channel_flow += channel_variable_tupledict[1,t_2]
                elif t <= time_horizon:
                    for k in range(channel_time):
                        channel_flow += channel_variable_tupledict[0,t-k]
                        channel_flow += channel_variable_tupledict[1,t-k]
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
        self.reverse_vertex_dict = {} # This dictionary assigns an index to a node
        self.green_vars = []
        self.blue_vars = []

        for node_index,node_tuple in enumerate(self.vertices):
            node = node_tuple[0]
            node_capacity = node_tuple[1]
            self.vertex_dict[node] = node_index #this is not fundamentally useful but more rigorous
            self.reverse_vertex_dict[node_index] = node

            # Create green variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.green_vars += [self.m.addVars(time_horizon, lb=0.0, ub=node_capacity+0.1, vtype=GRB.INTEGER,
                           name="green_" + str(node_index))]

            # Create blue variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.blue_vars += [self.m.addVars(time_horizon, lb=0.0, ub=node_capacity + 0.1, vtype=GRB.INTEGER,
                           name="blue_" + str(node_index))]

        self.m.update()

        # ------------------------------------------------------------------------------------------------------------ #

        # ----------------------------------- Source and sink matrix creation ---------------------------------------- #
        # We do this here since now we have an indexing for nodes and we can incorporate our source/sink terms in 2x2
        # matrices.
        self.source_matrix = np.zeros((len(self.vertices), time_horizon))
        self.sink_matrix = np.zeros((len(self.vertices), time_horizon))

        for source_term in sources:
            # source_term is a tuple (node,num_robots,time)
            source_node_index = self.vertex_dict[source_term[0]]
            self.source_matrix[source_node_index,source_term[2]] = source_term[1]

        # For objective purposes let's also store the sink term indexes and num_robots in a dict
        sink_dict={}
        for sink_term in sinks:
            # source_term is a tuple (node,num_robots,time)
            sink_node_index = self.vertex_dict[sink_term[0]]
            self.sink_matrix[sink_node_index,sink_term[2]] = sink_term[1]
            sink_dict[sink_node_index] = sink_term[1]
        # ------------------------------------------------------------------------------------------------------------ #

        # ----------------------------------------- FLOW CONSTRAINTS ------------------------------------------------- #
        for node_tuple in self.vertices:
            node = node_tuple[0]

            # We get the node index in order to be able to access blue and green variables
            node_index = self.vertex_dict[node]

            # We get the indexes of all neighboring channels to access the gadget variables
            list_of_channel_indexes = self.get_neighbor_channel_indexes(node)

            for t in range(time_horizon):

                # -------------------------------- Departure -------------------------------------- #
                # Let's start with all the constraints at t0 which will include the source term(s)
                # We don't have any time problem here since everybody departs at all time steps until the horizon.
                departure_constraint = LinExpr()
                departure_constraint += self.green_vars[node_index][t]
                if t>0:
                    departure_constraint -= self.blue_vars[node_index][t-1]
                for ch_t in list_of_channel_indexes:
                    ch_idx = ch_t[0]
                    departure_indicator = ch_t[1]
                    departure_constraint += self.channel_vars[ch_idx][departure_indicator,t]

                self.m.addConstr(departure_constraint == self.source_matrix[node_index,t],
                                 name="departure_constraint_node"+str(node_index)+"_t"+str(t))
                # ---------------------------------------------------------------------------------- #

                # ---------------------------------- Arrival --------------------------------------- #
                # All channels will not be treated equally according to their travel time.
                # If travel time > t, we don't consider the arrival.
                # With time 0, we look at the arrival at t+1, but the variables we are looking at still concern time t.
                arrival_constraint = LinExpr()

                # We keep negative on blue here to have a positive sign for sink:
                # if sink, then what arrives needs to be bigger than what leaves, so positive on green and channels.
                arrival_constraint -= self.blue_vars[node_index][t] #We do want there to be blue at t_horizon: ok.
                arrival_constraint += self.green_vars[node_index][t]

                for ch_t in list_of_channel_indexes:
                    t_channel = ch_t[2]
                    if t_channel <= t-1: #Consider t=0: we consider arrivals of channels of duration t_channel=1.
                        ch_idx = ch_t[0]
                        arrival_indicator = 1-ch_t[1] # We want arrivals, not departures!
                        arrival_constraint += self.channel_vars[ch_idx][arrival_indicator,t-t_channel+1]

                self.m.addConstr(arrival_constraint == self.sink_matrix[node_index, t],
                                 name="arrival_constraint_node" + str(node_index) + "_t" + str(t+1))

                # ---------------------------------------------------------------------------------- #

        self.m.update()

        # ---------------------------------- OBJECTIVE FUNCTION ------------------------------- #

        objective = LinExpr()

        # Let's say we want to minimize arrival time. Then we want k robots to be at k locations as early as possible.
        # So let's reward the blue (later: times its priority) at the sink points according to the number of
        # robots we want to see there.
        # Let's define new integer variables, one for each sink node.
        # This variable increases along time, is smaller than the current number of robots on the node at each time,
        # and is smaller than the sink term. We will attempt to maximize this variable.
        self.y_vars = []
        for sink_node_idx in sink_dict.keys():
            num_robots = sink_dict[sink_node_idx]

            # Variable creation and smaller than sink constraint
            ys = self.m.addVars(time_horizon,lb=0,ub=num_robots+0.1,vtype=GRB.INTEGER,name="ys_sink_node_"+str(sink_node_idx))
            self.y_vars += [ys]

            self.m.update()

            for t in range(time_horizon-1):
                objective += ys[t]

                # Smaller than number of robots constraint:
                self.m.addConstr(ys[t] <= self.blue_vars[sink_node_idx][t])

                # Increasing constraint:
                if t < time_horizon-1:
                    self.m.addConstr(ys[t+1]- ys[t] >= 0)
        # --------------------------------------------------------------------------------------- #

        self.m.update()
        self.m.setObjective(objective, GRB.MAXIMIZE)
        self.m.update()

        #embed()

        print("Model fully defined. Starting optimization")
        self.m.optimize()
        print("Done optimizing.")
        print("Advance over predicted arrival time: "+str(self.m.objVal))
        #embed()

    def create_perturbed_flow_problem(self,sources=[((1,1),2,0)],sinks=[((3,6),1,19),((8,5),1,19)],time_horizon=20):
        """sources/sinks is a list of tuples (vertex,number of robots coming/going,time_index)
        time_horizon should be equal to the max time in the sinks list. Let's not spend time looking for
        the max and add it as input to the function directly"""

        #Get number of robots. We only go through all the robots when we set the constraints, not when we create the model.
        num_robots = 0
        for k in range(len(sources)):
            num_robots += sources[k][1]

        #Create the gurobi model
        self.m = Model("Factory")


        # Initialize an empty dictionary
        self.flow_graph = {}

        self.time_horizon = time_horizon


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

            for t in range(1,time_horizon+channel_time): #We end at T + channel time because the last departures start at T
                channel_flow = LinExpr()
                if t < channel_time:
                    for t_2 in range(t+1):
                        channel_flow += channel_variable_tupledict[0,t_2]
                        channel_flow += channel_variable_tupledict[1,t_2]
                elif t <= time_horizon:
                    for k in range(channel_time):
                        channel_flow += channel_variable_tupledict[0,t-k]
                        channel_flow += channel_variable_tupledict[1,t-k]
                else:
                    for t_2 in range(t-time_horizon):
                        channel_flow += channel_variable_tupledict[0,time_horizon+t_2]
                        channel_flow += channel_variable_tupledict[1,time_horizon+t_2]

                # We add stochasticity in the considered graph by creating a variable, stochastic_obstacle,
                # that takes value 0 with probability 1/5: at one time step, one channel has 20% chance being blocked.
                p = np.random.uniform(0.0,1.0)
                stochastic_obstacle=0
                if p >= 0.2:
                    stochastic_obstacle=1

                self.m.addConstr(channel_flow <= channel_capacity*stochastic_obstacle,name="channel_constraint_"+str(channel_index)+"_t"+str(t))
        self.m.update()

        # ------------------------------------------------------------------------------------------------------------ #


        # --------------------------- STATIONARY EDGE VARIABLES AND CONSTRAINTS -------------------------------------- #

        # Green and blue arrows in the paper
        self.vertex_dict = {} # This dictionary assigns a node to an index
        self.reverse_vertex_dict = {} # This dictionary assigns an index to a node
        self.green_vars = []
        self.blue_vars = []

        for node_index,node_tuple in enumerate(self.vertices):
            node = node_tuple[0]
            node_capacity = node_tuple[1]
            self.vertex_dict[node] = node_index #this is not fundamentally useful but more rigorous
            self.reverse_vertex_dict[node_index] = node

            # Create green variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.green_vars += [self.m.addVars(time_horizon, lb=0.0, ub=node_capacity+0.1, vtype=GRB.INTEGER,
                           name="green_" + str(node_index))]

            # Create blue variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.blue_vars += [self.m.addVars(time_horizon, lb=0.0, ub=node_capacity + 0.1, vtype=GRB.INTEGER,
                           name="blue_" + str(node_index))]

        self.m.update()

        # ------------------------------------------------------------------------------------------------------------ #

        # ----------------------------------- Source and sink matrix creation ---------------------------------------- #
        # We do this here since now we have an indexing for nodes and we can incorporate our source/sink terms in 2x2
        # matrices.
        self.source_matrix = np.zeros((len(self.vertices), time_horizon))
        self.sink_matrix = np.zeros((len(self.vertices), time_horizon))

        for source_term in sources:
            # source_term is a tuple (node,num_robots,time)
            source_node_index = self.vertex_dict[source_term[0]]
            self.source_matrix[source_node_index,source_term[2]] = source_term[1]

        # For objective purposes let's also store the sink term indexes and num_robots in a dict
        sink_dict={}
        for sink_term in sinks:
            # source_term is a tuple (node,num_robots,time)
            sink_node_index = self.vertex_dict[sink_term[0]]
            self.sink_matrix[sink_node_index,sink_term[2]] = sink_term[1]
            sink_dict[sink_node_index] = sink_term[1]
        # ------------------------------------------------------------------------------------------------------------ #

        # ----------------------------------------- FLOW CONSTRAINTS ------------------------------------------------- #
        for node_tuple in self.vertices:
            node = node_tuple[0]

            # We get the node index in order to be able to access blue and green variables
            node_index = self.vertex_dict[node]

            # We get the indexes of all neighboring channels to access the gadget variables
            list_of_channel_indexes = self.get_neighbor_channel_indexes(node)

            for t in range(time_horizon):

                # -------------------------------- Departure -------------------------------------- #
                # Let's start with all the constraints at t0 which will include the source term(s)
                # We don't have any time problem here since everybody departs at all time steps until the horizon.
                departure_constraint = LinExpr()
                departure_constraint += self.green_vars[node_index][t]
                if t>0:
                    departure_constraint -= self.blue_vars[node_index][t-1]
                for ch_t in list_of_channel_indexes:
                    ch_idx = ch_t[0]
                    departure_indicator = ch_t[1]
                    departure_constraint += self.channel_vars[ch_idx][departure_indicator,t]

                self.m.addConstr(departure_constraint == self.source_matrix[node_index,t],
                                 name="departure_constraint_node"+str(node_index)+"_t"+str(t))
                # ---------------------------------------------------------------------------------- #

                # ---------------------------------- Arrival --------------------------------------- #
                # All channels will not be treated equally according to their travel time.
                # If travel time > t, we don't consider the arrival.
                # With time 0, we look at the arrival at t+1, but the variables we are looking at still concern time t.
                arrival_constraint = LinExpr()

                # We keep negative on blue here to have a positive sign for sink:
                # if sink, then what arrives needs to be bigger than what leaves, so positive on green and channels.
                arrival_constraint -= self.blue_vars[node_index][t] #We do want there to be blue at t_horizon: ok.
                arrival_constraint += self.green_vars[node_index][t]

                for ch_t in list_of_channel_indexes:
                    t_channel = ch_t[2]
                    if t_channel <= t-1: #Consider t=0: we consider arrivals of channels of duration t_channel=1.
                        ch_idx = ch_t[0]
                        arrival_indicator = 1-ch_t[1] # We want arrivals, not departures!
                        arrival_constraint += self.channel_vars[ch_idx][arrival_indicator,t-t_channel+1]

                self.m.addConstr(arrival_constraint == self.sink_matrix[node_index, t],
                                 name="arrival_constraint_node" + str(node_index) + "_t" + str(t+1))

                # ---------------------------------------------------------------------------------- #

        self.m.update()

        # ---------------------------------- OBJECTIVE FUNCTION ------------------------------- #

        objective = LinExpr()

        # Let's say we want to minimize arrival time. Then we want k robots to be at k locations as early as possible.
        # So let's reward the blue (later: times its priority) at the sink points according to the number of
        # robots we want to see there.
        # Let's define new integer variables, one for each sink node.
        # This variable increases along time, is smaller than the current number of robots on the node at each time,
        # and is smaller than the sink term. We will attempt to maximize this variable.
        self.y_vars = []
        for sink_node_idx in sink_dict.keys():
            num_robots = sink_dict[sink_node_idx]

            # Variable creation and smaller than sink constraint
            ys = self.m.addVars(time_horizon,lb=0,ub=num_robots+0.1,vtype=GRB.INTEGER,name="ys_sink_node_"+str(sink_node_idx))
            self.y_vars += [ys]

            self.m.update()

            for t in range(time_horizon-1):
                objective += ys[t]

                # Smaller than number of robots constraint:
                self.m.addConstr(ys[t] <= self.blue_vars[sink_node_idx][t])

                # Increasing constraint:
                if t < time_horizon-1:
                    self.m.addConstr(ys[t+1]- ys[t] >= 0)
        # --------------------------------------------------------------------------------------- #

        self.m.update()
        self.m.setObjective(objective, GRB.MAXIMIZE)
        self.m.update()

        #embed()

        print("Model fully defined. Starting optimization")
        self.m.optimize()
        print("Done optimizing.")
        print("Advance over predicted arrival time: "+str(self.m.objVal))
        #embed()

    def find_robust_solution(self,coeffs,sources=[((1,1),2,0)],sinks=[((3,6),1,19),((8,5),1,19)],time_horizon=20):
        """sources/sinks is a list of tuples (vertex,number of robots coming/going,time_index)
        time_horizon should be equal to the max time in the sinks list. Let's not spend time looking for
        the max and add it as input to the function directly"""

        #Get number of robots. We only go through all the robots when we set the constraints, not when we create the model.
        num_robots = 0
        for k in range(len(sources)):
            num_robots += sources[k][1]

        #Create the gurobi model
        self.m = Model("Factory")


        # Initialize an empty dictionary
        self.flow_graph = {}

        self.time_horizon = time_horizon


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

                    ##Now let's add to the channel dictionary indexed by edge time
                    #try:
                    #    self.channel_by_time_dict[t_edge] += [copy.deepcopy(channel_counter)]
                    #except:
                    #    self.channel_by_time_dict[t_edge] = [copy.deepcopy(channel_counter)]


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

            for t in range(1,time_horizon+channel_time): #We end at T + channel time because the last departures start at T
                channel_flow = LinExpr()
                if t < channel_time:
                    for t_2 in range(t+1):
                        channel_flow += channel_variable_tupledict[0,t_2]
                        channel_flow += channel_variable_tupledict[1,t_2]
                elif t <= time_horizon:
                    for k in range(channel_time):
                        channel_flow += channel_variable_tupledict[0,t-k]
                        channel_flow += channel_variable_tupledict[1,t-k]
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
        self.reverse_vertex_dict = {} # This dictionary assigns an index to a node
        self.green_vars = []
        self.blue_vars = []

        for node_index,node_tuple in enumerate(self.vertices):
            node = node_tuple[0]
            node_capacity = node_tuple[1]
            self.vertex_dict[node] = node_index #this is not fundamentally useful but more rigorous
            self.reverse_vertex_dict[node_index] = node

            # Create green variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.green_vars += [self.m.addVars(time_horizon, lb=0.0, ub=node_capacity+0.1, vtype=GRB.INTEGER,
                           name="green_" + str(node_index))]

            # Create blue variables for one node and all times until time horizon. The time step here is 1
            # The constraint is automatically set with the upper bound
            self.blue_vars += [self.m.addVars(time_horizon, lb=0.0, ub=node_capacity + 0.1, vtype=GRB.INTEGER,
                           name="blue_" + str(node_index))]

        self.m.update()

        # ------------------------------------------------------------------------------------------------------------ #

        # ----------------------------------- Source and sink matrix creation ---------------------------------------- #
        # We do this here since now we have an indexing for nodes and we can incorporate our source/sink terms in 2x2
        # matrices.
        self.source_matrix = np.zeros((len(self.vertices), time_horizon))
        self.sink_matrix = np.zeros((len(self.vertices), time_horizon))

        for source_term in sources:
            # source_term is a tuple (node,num_robots,time)
            source_node_index = self.vertex_dict[source_term[0]]
            self.source_matrix[source_node_index,source_term[2]] = source_term[1]

        # For objective purposes let's also store the sink term indexes and num_robots in a dict
        sink_dict={}
        for sink_term in sinks:
            # source_term is a tuple (node,num_robots,time)
            sink_node_index = self.vertex_dict[sink_term[0]]
            self.sink_matrix[sink_node_index,sink_term[2]] = sink_term[1]
            sink_dict[sink_node_index] = sink_term[1]
        # ------------------------------------------------------------------------------------------------------------ #

        # ----------------------------------------- FLOW CONSTRAINTS ------------------------------------------------- #
        for node_tuple in self.vertices:
            node = node_tuple[0]

            # We get the node index in order to be able to access blue and green variables
            node_index = self.vertex_dict[node]

            # We get the indexes of all neighboring channels to access the gadget variables
            list_of_channel_indexes = self.get_neighbor_channel_indexes(node)

            for t in range(time_horizon):

                # -------------------------------- Departure -------------------------------------- #
                # Let's start with all the constraints at t0 which will include the source term(s)
                # We don't have any time problem here since everybody departs at all time steps until the horizon.
                departure_constraint = LinExpr()
                departure_constraint += self.green_vars[node_index][t]
                if t>0:
                    departure_constraint -= self.blue_vars[node_index][t-1]
                for ch_t in list_of_channel_indexes:
                    ch_idx = ch_t[0]
                    departure_indicator = ch_t[1]
                    departure_constraint += self.channel_vars[ch_idx][departure_indicator,t]

                self.m.addConstr(departure_constraint == self.source_matrix[node_index,t],
                                 name="departure_constraint_node"+str(node_index)+"_t"+str(t))
                # ---------------------------------------------------------------------------------- #

                # ---------------------------------- Arrival --------------------------------------- #
                # All channels will not be treated equally according to their travel time.
                # If travel time > t, we don't consider the arrival.
                # With time 0, we look at the arrival at t+1, but the variables we are looking at still concern time t.
                arrival_constraint = LinExpr()

                # We keep negative on blue here to have a positive sign for sink:
                # if sink, then what arrives needs to be bigger than what leaves, so positive on green and channels.
                arrival_constraint -= self.blue_vars[node_index][t] #We do want there to be blue at t_horizon: ok.
                arrival_constraint += self.green_vars[node_index][t]

                for ch_t in list_of_channel_indexes:
                    t_channel = ch_t[2]
                    if t_channel <= t-1: #Consider t=0: we consider arrivals of channels of duration t_channel=1.
                        ch_idx = ch_t[0]
                        arrival_indicator = 1-ch_t[1] # We want arrivals, not departures!
                        arrival_constraint += self.channel_vars[ch_idx][arrival_indicator,t-t_channel+1]

                self.m.addConstr(arrival_constraint == self.sink_matrix[node_index, t],
                                 name="arrival_constraint_node" + str(node_index) + "_t" + str(t+1))

                # ---------------------------------------------------------------------------------- #

        self.m.update()

        # ---------------------------------- OBJECTIVE FUNCTION ------------------------------- #

        objective = LinExpr()
        for k,var in enumerate(self.m.getVars()):
            objective += coeffs[k+1][0]*var #We don't need the first constant coeff in the objective since we are minimizing

        # --------------------------------------------------------------------------------------- #

        self.m.update()
        self.m.setObjective(objective, GRB.MINIMIZE)
        self.m.update()

        #embed()

        print("Model fully defined. Starting optimization")
        self.m.optimize()
        print("Done optimizing.")
        print("Advance over predicted arrival time: "+str(self.m.objVal))
        #embed()


    def get_neighbor_channel_indexes(self,node):
        # First we go through the original dictionary of edges
        channel_indexes = []

        for channel_tuple in self.channel_dict.keys():
            node1 = channel_tuple[0]
            node2 = channel_tuple[1]
            if node1 == node:
                ch_idx = self.channel_dict[channel_tuple][0]
                t_edge = self.channel_dict[channel_tuple][1]
                channel_indexes += [(ch_idx,0,t_edge)] #Your node is node1 so take channel var 0 as the departing
            if node2 == node:
                ch_idx = self.channel_dict[channel_tuple][0]
                t_edge = self.channel_dict[channel_tuple][1]
                channel_indexes += [(ch_idx,1,t_edge)]

        return channel_indexes

    def solution_to_labeled_path(self):
        """From a solution this returns the corresponding sequence of nodes and edges for each robot to take.
        Note that there are several possible xs for one possible solution, and that we can't trace back to the path
        from the output of this function."""
        solution = []

        # The way we will extract paths here is by tracking each agent from the beginning.

        # First, we get the starting positions by scanning the source matrix.
        for kv in range(np.shape(self.source_matrix)[0]):
            for kt in range(np.shape(self.source_matrix)[1]):
                if self.source_matrix[kv][kt] != 0:
                    path = self.extract_path_from(kv,kt)
                    solution += [path]

        return solution

    def extract_path_from(self,vertex_index,time_index):
        """Note that vertex index is not the id of the vertex and that vertex_dict and its reciprocal are here to
        manage that."""
        path = []

        v_idx = vertex_index
        t = time_index


        #we are looking for the first path that starts with vertex v after time t
        while True:

            # Prepare variables
            current_vertex = self.vertices[v_idx]
            node_id = current_vertex[0]
            capacity = current_vertex[1]
            count = 0

            # If the capacity is >= 2 we may have dealt with the following path before
            if capacity > 1:
                try:
                    count = self.occupancy_dict[(v_idx, t)]
                except:
                    count = 0
                self.occupancy_dict[(v_idx, t)] = count



            # We need v_idx and t (or just t) to be updated at each iteration.
            try:


                #Let's start by scanning the next possibilities for channels
                # Inefficient method:
                found = False
                for(ch_idx,channel) in enumerate(self.channel_dict.keys()):

                    if channel[0] == node_id:
                        channel_index = self.channel_dict[channel][0]
                        channel_var = self.channel_vars[channel_index][0,t].x #var 0: from u to v

                        if channel_var > 0: #At least an agent enters this edge at t
                            if count < channel_var: #We have counted k already but there are more than k occupants here
                                path += [(channel[0],channel[1])]
                                count += 1
                                try:
                                    self.occupancy_dict[(v_idx, t)] = self.occupancy_dict[(v_idx, t)] + 1
                                except:
                                    pass

                                t_edge = self.channel_dict[channel][1]

                                # Our next search only needs to be at t + t_edge
                                t += t_edge
                                #Next node to look at
                                v_idx = self.vertex_dict[channel[1]]

                                #Get out of the inner for loop
                                found = True
                                break
                            else:
                                #We have counted k, here we find m robots leaving with m < k.
                                count += channel_var

                    elif channel[1] == node_id: #Reverse direction
                        channel_index = self.channel_dict[channel][0]
                        channel_var = self.channel_vars[channel_index][1, t].x  # var 1: from v to u

                        if channel_var > 0:  # At least an agent enters this edge at t
                            if count < channel_var:  # We have counted k already but there are more than k occupants here
                                path += [(channel[1], channel[0])]
                                count += 1
                                try:
                                    self.occupancy_dict[(v_idx, t)] = self.occupancy_dict[(v_idx, t)] + 1
                                except:
                                    pass

                                t_edge = self.channel_dict[channel][1]

                                # Our next search only needs to be at t + t_edge
                                t += t_edge
                                # Next node to look at
                                v_idx = self.vertex_dict[channel[0]] #The next vertex to look at is the first

                                # Get out of the inner for loop
                                found = True
                                break
                            else:
                                #We have counted k, here we find m robots leaving with m < k.
                                count += channel_var
                    # ----------------------------------------------------------------------------------------- #
                # If we arrive here, we searched through all the channels.
                if found == False:
                    t += 1 #Search at the following time step






            except:
                break #This means that we are looking so much ahead in time that we can't access time index t anymore

        return path

    def simulate_particles(self,solution,save=False):
        """This function only works on a grid! Reason: we are able to do collision checking and delay the robots in function.
        If we had 3s edges we won't know whether we can send our agent without it colliding at t+3"""
        cost = 0
        situation_t0 = [solution[agent][0][0] for agent in range(len(solution))] # All nodes at t0
        finished = False
        t = 0
        situation_t = [[] for k in range(len(solution))]
        final_situation = [solution[agent][-1][1] for agent in range(len(solution))]

        solution_poppable = copy.deepcopy(solution)

        while finished == False:
            done = 0
            # Go through all of the robots one by one at random and try to get them to the next step
            indexes = [k for k in range(len(solution))]
            np.random.shuffle(indexes)
            for agent_index in indexes:
                if len(solution_poppable[agent_index]) >= 1:
                    agent_next_vertex = solution_poppable[agent_index][0][1]
                else:
                    #assert (solution[agent_index][-1][1] == situation_t[agent_index]), embed() not to use anymore
                    done += 1
                    continue
                #First, with probability 1/5 we are not able to make it
                p = np.random.uniform(0.0,1.0)
                if p < 0.2:
                    situation_t[agent_index] = situation_t0[agent_index]
                    cost += 1
                else:
                    #Then we need to check whether there is someone at the node already
                    possible = True
                    for vertex in situation_t0:
                        if agent_next_vertex == vertex:
                            # To avoid deadlocks, for a situation that looks impossible, we will make it possible with probability 1/3
                            possible = False
                            p = np.random.uniform(0.0, 1.0)
                            if p < 0.33:
                                possible = True
                    if possible: # Advance
                        situation_t[agent_index] = agent_next_vertex
                        cost += 1
                        # Remove edge just traversed from list
                        del solution_poppable[agent_index][0]
                        # Are we finished now?
                        if solution[agent_index][-1][1] == agent_next_vertex:
                            # Remove the agent from circulation because it is done
                            situation_t[agent_index] = []
                            done += 1
                    else:
                        situation_t[agent_index] = situation_t0[agent_index]
                        cost += 1
            # Prepare for next iteration
            t += 1
            situation_t0 = situation_t

            #Check for termination
            if done >= len(solution):
                finished = True

        return cost

    def get_solution_cost(self,num_particles=100):
        """Tests num_particles possible scenarios for this solution and returns the average cost per scenario"""
        solution = self.solution_to_labeled_path()
        cost = 0
        for k in range(num_particles):
            cost += self.simulate_particles(solution)
        cost = float(cost)/float(num_particles)
        return solution,cost

    def get_x(self):
        x = []
        for v in self.m.getVars():
            x += [v.x]
        return x



def create_graph_factory1():
    my_graph = graph()
    edges = [((1,1),(1,3),3,3),   ((1,3),(3,6),3,3),  ((1,3),(4,3),3,1),  ((1,3),(4,3),2,1),  ((3,6),(13,6),5,4),  ((13,6),(8,5),3,10),
             ((8,5),(4,3),3,3),  ((4,3),(13,6),8,8) ]
    list_of_vertices = [((1,1),4),((1,3),2),((3,6),1),((4,3),3),((13,6),5),((8,5),1)]
    my_graph.add_edges_implicitly_bidirectional(edges)
    my_graph.define_vertices(list_of_vertices)
    return my_graph

def create_dense_graph():
    """Creates 10x10 grid graph"""
    my_graph = graph()
    list_of_vertices = []
    edges = []
    for x in range(1,11):
        for y in range(1,11):
            list_of_vertices += [((x,y),1)]
            if x != 10:
                right_edge = ((x,y),(x+1,y),1,1)
                edges += [right_edge]
            if y != 10:
                upper_edge = ((x,y),(x,y+1),1,1)
                edges += [upper_edge]
    my_graph.add_edges_implicitly_bidirectional(edges)
    my_graph.define_vertices(list_of_vertices)
    return my_graph
