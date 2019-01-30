# Here is where the graph object is created.
import matplotlib.pyplot as plt

class graph():
    """ This unidirected graph has vertexes and edges. Each edge has a cost and a capacity.
    self.g = { vertex: {vertex2: [(cost1,capacity1),(cost2,capacity2)]}, nextvertex: ... }
    a vertex is a tuple of integers (x,y)"""
    def __init__(self):
        self.g = {}

    def add_edges(self, edges):
        """ edges is a list of tuples (vertex1, vertex2, cost, capacity)"""
        for edge in edges:
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


    def plot_in_factory(self):
        nodes = self.g.keys()
        for node in nodes:
            x = node[0]
            y = node[1]
            plt.scatter(x, y, s=20, marker=(5, 0, 180), c='b')

def create_graph_factory1():
    my_graph = graph()
    edges =
