# Here is where the graph object is created.
import matplotlib.pyplot as plt

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


    def plot_in_factory(self):
        nodes = self.g.keys()
        for node in nodes:
            x = node[0] + 0.5
            y = node[1] + 0.5
            plt.scatter(x, y, s=20, marker=(5, 0, 180), c='b')



def create_graph_factory1():
    my_graph = graph()
    edges = [((1,1),(1,3),3,3),   ((1,3),(3,6),3,3),  ((1,3),(4,3),3,1),  ((1,3),(4,3),2,1),  ((3,6),(13,6),5,4),  ((13,6),(10,5),3,10),
             ((10,5),(4,3),3,3),  ((4,3),(13,6),8,8) ]
    my_graph.add_edges_bidirectional(edges)
    return my_graph
