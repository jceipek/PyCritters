

import networkx as nx

class Morphology(object):
    
    def __init__(self):
        self.graph = nx.MultiGraph()
        self.graph.ad