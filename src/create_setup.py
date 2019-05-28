## Here is where you create a setup to work with
import matplotlib.pyplot as plt
from matplotlib.contour import ContourSet
import matplotlib.cm as cm
import numpy as np
from Graph import graph, create_graph_factory1, create_dense_graph
from IPython import embed

class factory():
    def __init__(self,graph=None,machine_sinks=[]):
        self.bounds = np.array([0,30,0,10])
        self.machines = []
        for sink in machine_sinks:
            self.machines += [machine([sink[0][0],sink[0][1]])]
        #self.machines = [machine(),machine([8,4])]
        self.robots = [robot([0,0]),robot([0,1])]

        #lines0 = [[[0, 0], [0, 4]]]
        #lines1 = [[[4, 6], [6, 6], [12, 6]]]
        #lines2 = [[[3, 0], [3, 2]], [[3, 3], [3, 4]]]
        line_bounds = [[[self.bounds[0],self.bounds[2]],[self.bounds[0],self.bounds[3]],[self.bounds[1],self.bounds[3]],[self.bounds[1],self.bounds[2]],[self.bounds[0],self.bounds[2]]]]
        #self.walls = [lines0,lines1,lines2,line_bounds]
        self.walls = [line_bounds]

        #This is the graph object we will deal with
        if graph == None:
            self.mrpp_graph = create_graph_factory1()
        else:
            self.mrpp_graph = graph

    def plot_floor(self,graph=False,lines_to_plot=[]):

        # the axes attributes need to be set before the call to subplot
        fig = plt.figure()

        if graph:
            plt.subplot(2,1,1)

        plt.title('Factory')

        plt.axis('scaled')
        plt.xlim((self.bounds[0], self.bounds[1]))
        plt.ylim((self.bounds[2],self.bounds[3]))
        plt.xticks(np.arange(self.bounds[0], self.bounds[1]+1, step=1))
        plt.yticks(np.arange(self.bounds[2], self.bounds[3]+1, step=1))
        plt.grid(True)

        self.plot_machines()
        self.plot_walls()
        self.plot_robots()

        for line_tp in lines_to_plot:
            x = line_tp[0][0] + 0.5
            y = line_tp[0][1] + 0.5
            xp = line_tp[1][0] + 0.5
            yp = line_tp[1][1] + 0.5
            line_to_plot = [(x, xp), (y, yp), 'r']
            plt.plot(*line_to_plot)



        if graph:
            plt.subplot(2,1,2)
            plt.axis('scaled')
            plt.xlim((self.bounds[0], self.bounds[1]))
            plt.ylim((self.bounds[2], self.bounds[3]))
            plt.xticks(np.arange(self.bounds[0], self.bounds[1] + 1, step=1))
            plt.yticks(np.arange(self.bounds[2], self.bounds[3] + 1, step=1))
            plt.grid(False)

            self.plot_machines()
            self.plot_walls()

            self.mrpp_graph.plot_in_factory()


        #plt.show()

    def plot_machines(self):
        # -------------------- Plotting machines in red ---------------------- #
        for mymachine in self.machines:
            m = ContourSet(plt.gca(), [0, 1], [[mymachine.shape]], filled=True, colors='r')
            plt.scatter(mymachine.access_point[0], mymachine.access_point[1], s=10, marker=(3, 0, 180), c='r')
        # -------------------------------------------------------------------- #

    def plot_walls(self):
        # Contour lines (non-filled).
        lines = ContourSet(plt.gca(), np.arange(len(self.walls)), self.walls,
                           linewidths=3, colors='k')

    def plot_robots(self):
        # -------------------- Plotting robots in green ---------------------- #
        for myrobot in self.robots:
            m = ContourSet(plt.gca(), [0, 1], [[myrobot.shape]], filled=True, colors=myrobot.color)
        # -------------------------------------------------------------------- #

    def plot_flow_solution(self,savefig=True):
        for t in range(self.mrpp_graph.time_horizon):
            self.robots = []
            for node_index in range(len(self.mrpp_graph.vertices)):
                concerned_node = self.mrpp_graph.reverse_vertex_dict[node_index]
                num_robots_in = self.mrpp_graph.blue_vars[node_index][t].x + self.mrpp_graph.sink_matrix[node_index,t]
                if num_robots_in == 1:
                    self.robots += [robot(location=concerned_node,color='g')]
                elif num_robots_in == 2:
                    self.robots += [robot(location=concerned_node,color='b')]
            mylines =[]
            for channel_tuple in self.mrpp_graph.channel_dict.keys():
                channel_index = self.mrpp_graph.channel_dict[channel_tuple][0]
                channel_time = self.mrpp_graph.channel_dict[channel_tuple][1]
                channel_occupancy = 0
                for t_ch in range(channel_time-1):
                    if (t-t_ch) >= 0:
                        channel_occupancy += self.mrpp_graph.channel_vars[channel_index][0,t-t_ch].x
                        channel_occupancy += self.mrpp_graph.channel_vars[channel_index][1,t-t_ch].x
                if channel_occupancy > 0:
                    node1 = channel_tuple[0]
                    node2 = channel_tuple[1]
                    mylines += [[node1,node2]]


            self.plot_floor(graph=False,lines_to_plot = mylines)

            plt.text(19,5,"time: "+str(t))
            if savefig:
                plt.savefig("Plot_results/7/" + str(t) + ".png")
            plt.close()


class obstacle():
    def __init__(self,shape,location):
        self.location = location
        self.create_polygon(shape,location) #creates self.shape

    def create_polygon(self,shape,location):
        x = location[0]
        y = location[1]
        if shape[0] == "rectangle":
            dx = shape[1][0]
            dy = shape[1][1]
            self.shape = [[x,y],[x,y+dy],[x+dx,y+dy],[x+dx,y]]
        elif shape[0] == "circle":
            rad = shape[1]/2.
            cx = x+rad
            cy = y+rad
            polygon = []
            for k in range(15):
                theta = 2.*np.pi*k/15.
                polygon += [[cx + rad*np.cos(theta),cy + rad*np.sin(theta)]]
            self.shape = polygon
        return

class machine(obstacle):
    def __init__(self,location=[3,5]):
        shape = ["rectangle",[1,1]]
        obstacle.__init__(self, shape,location)
        self.access_point = [self.location[0]+0.5,self.location[1]+1.5]

class wall(obstacle):
    def __init__(self):
        shape = ["contour", [1, 1]]
        location = [3, 5]
        obstacle.__init__(self, shape, location)

class robot(obstacle):
    def __init__(self,location=[3,5],color='g'):
        shape = ["circle", 1]
        obstacle.__init__(self, shape, location)
        self.color = color

