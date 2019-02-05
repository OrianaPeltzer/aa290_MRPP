# Here is a setup and some associated functions
# Setup 1 is a small factory.

import matplotlib.pyplot as plt
from matplotlib.contour import ContourSet
import matplotlib.cm as cm
import numpy as np
from Graph import graph, create_graph_factory1
from IPython import embed

class factory():
    def __init__(self):
        self.robots = None
        self.bounds = np.array([0,30,0,10])
        self.machines = [machine(),machine([8,4])]
        self.robots = [robot([0,0]),robot([0,1])]

        lines0 = [[[0, 0], [0, 4]]]
        lines1 = [[[4, 6], [6, 6], [12, 6]]]
        lines2 = [[[3, 0], [3, 2]], [[3, 3], [3, 4]]]
        line_bounds = [[[self.bounds[0],self.bounds[2]],[self.bounds[0],self.bounds[3]],[self.bounds[1],self.bounds[3]],[self.bounds[1],self.bounds[2]],[self.bounds[0],self.bounds[2]]]]
        self.walls = [lines0,lines1,lines2,line_bounds]

        #This is the graph object we will deal with
        self.mrpp_graph = create_graph_factory1()

    def plot_floor(self,graph=False):

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


        plt.show()

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
            m = ContourSet(plt.gca(), [0, 1], [[myrobot.shape]], filled=True, colors='g')
        # -------------------------------------------------------------------- #


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
    def __init__(self,location=[3,5]):
        shape = ["circle", 1]
        obstacle.__init__(self, shape, location)

if __name__ == "__main__":
    Factory = factory()
    Factory.plot_floor(graph=True)
    Factory.mrpp_graph.create_flow_problem()
    embed()