import networkx as nx
import neural
import math
import copy

from random import random

class Morphology(object):
    
    def __init__(self):
        self.graph = nx.MultiDiGraph()
        
    def addNode(self, node):
        self.graph.add_node(node)
        
    def addConnection(self, connection):
        for node in connection.nodes: self.addNode(node)
        self.graph.add_edge(*connection.nodes, key=connection.id, 
                            connection=connection)
        
    def createConnection(self, *nodes): 
        #XXX Do you really want new nodes to be created every time
        #we add a connection? - Julian
        # They aren't... the graph.add_node function does nothing if the node 
        # is already in the graph. Its just a safe call to catch a case where
        # a connection is added between nodes not already in the graph.
        # Maybe change this to an error? - Chase
        self.addConnection(MorphConnection(tuple(nodes)))
        
    @property
    def nodes(self): 
        return self.graph.nodes_iter()
    
    @property
    def connections(self):
        for _, _, data in self.graph.edges_iter(data=True):
            yield data['connection']
            
    def removeNode(self, node):
        """Removes a node and its connections from the morphology
        """
        self.graph.remove_node(node)
        
    def clone(self):
        newNN = copy.deepcopy(self)
        return newNN
    
    def expand(self):
        cache = {}
        def expandNode(node, depth):
            key = node, depth
            try:
                return copy.deepcopy(cache[key])
            except KeyError: pass
            
            graph = nx.Graph()
            root = copy.deepcopy(node)
            graph.add_node(root)
            
            for _, neighbor, data in self.graph.out_edges_iter(data=True):
                connection = data['connection']
                
                if node is neighbor:
                    subDepth = depth + 1
                    if connection.recursionLimit >= subDepth: continue
                else:
                    subDepth = 0
                
                subroot, subgraph = expandNode(neighbor, subDepth)
                
                graph.add_nodes_from(subgraph.nodes_iter())
                graph.add_edges_from(subgraph.edges_iter(data=True))
                
                graph.add_edge(root, subroot, { 'connection': connection })
            
            cache[key] = root, graph
        
        return expandNode(next(self.nodes), 0)[0]

class MorphNode(object):
    
    def __init__(self, width=1, height=1, depth=1, nn=None):
        self.width = width
        self.height = height
        self.depth = depth
        self.nn = nn or self._createNetwork()
        
    def _createNetwork(self):
        #TODO: random numbers here:
        return neural.randomNeuralNetwork(2, 2, 5)
    
        
HINGE_JOINT = 0
        
class MorphConnection(object):
    
    #Class  
    idCounter = 0
    
    def __init__(self, nodes, joint=HINGE_JOINT, actuators=None, 
                 locations=None, recursionLimit=1):
        self.id = MorphConnection.idCounter
        MorphConnection.idCounter += 1
        
        self.nodes = nodes
        self.joint = joint
        self.actuators = actuators or self._createActuators()
        self.locations = locations or self._createLocations()
        self.recursionLimit = recursionLimit
        
    def _createActuators(self):
        return Actuator(), Actuator()
    
    def _createLocations(self):
        def randomLocation(): return 2*math.pi*random()
        return randomLocation(), randomLocation()
    
class Actuator(object):
    
    def __init__(self, strength=None, limits=(0.0, 1.0)):
        self.strength = strength or random()
        self.limits = limits


def createSnake():
    """Hardcoded test function that creates a snake creature
    
    (for testing purposes)
    """
    
    snake = Morphology()
    
    head = MorphNode(1,1,1,nn=None)
    middle = MorphNode(1,1,1,nn=None)
    tail = MorphNode(1,1,1,nn=None)
    
    #Put these back in if the createConnection function
    #doesn't add nodes
    #snake.addNode(head) 
    #snake.addNode(middle)
    #snake.addNode(tail)
    
    snake.createConnection(self, head, middle)
    snake.createConnection(self, middle, tail)

    return snake
