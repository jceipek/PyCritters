import networkx as nx
import neural
import math
import copy
import graphs

from utils import *
from mutations import *
from random import random, choice

class Morphology(object):
    
    def __init__(self):
        self.graph = nx.MultiDiGraph()
        
    def addNode(self, node):
        # Note: if the node is already in the graph, it will
        # not be added again.
        self.graph.add_node(node)
        
    def addConnection(self, connection):
        for node in connection.nodes: self.addNode(node)
        self.graph.add_edge(*connection.nodes, key=connection.id, 
                            connection=connection)
        
    def createConnection(self, *nodes): 
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
        newMorph = copy.deepcopy(self)
        return newMorph
    
    def expand(self):
        cache = {}
        def expandNode(node, depth):
            key = node, depth
            try:
                return copy.deepcopy(cache[key])
            except KeyError: pass
            
            graph = nx.Graph()
            root = node.clone()
            graph.add_node(root)
            
            for _, neighbor, data in self.graph.out_edges_iter(node, data=True):
                connection = data['connection']
                
                if node == neighbor:
                    subDepth = depth + 1
                    if subDepth >= connection.recursionLimit: continue
                else:
                    subDepth = 0
                
                subroot, subgraph = expandNode(neighbor, subDepth)
                
                graph.add_nodes_from(subgraph.nodes_iter())
                graph.add_edges_from(subgraph.edges_iter(data=True))
                
                graph.add_edge(root, subroot, { 'connection': connection })
            
            value = root, graph
            cache[key] = value
            return value
        
        return expandNode(next(self.nodes), 0)[1]
    
    def mutate(self):
        newMorph = self.clone()
        graphs.mutate(newMorph.graph, MorphNode, MorphConnection)
        return newMorph
    
    def crossover(self, other):
        daughters = self.clone(), other.clone()
        graphs.crossover(daughters[0].graph, daughters[1].graph, 
                         MorphConnection)
        return daughters

class MorphNode(object):
    
    CROSSOVER_RATE = 0.4
    
    _dimensionsValue = MutableFloat(range=(0.1, 5.0), rate=0.1)
    
    def __init__(self, dimensions=None, nn=None):
        self.dimensions = dimensions or self._dimensionsValue(repeat=3)
        self.nn = nn or self._createNetwork()
        
    @property
    def width(self): return self.dimensions[0]
    
    @property
    def height(self): return self.dimensions[1]
    
    @property
    def depth(self): return self.dimensions[2]
    
    def mutate(self):
        self.dimensions = self._dimensionsValue(self.dimensions)
        self.nn = self.nn.mutate()
        
    def crossover(self, other):
        d1, d2 = self.clone(), other.clone()
        
        if random() < MorphNode.CROSSOVER_RATE:
            d1.dimensions, d2.dimensions = d2.dimensions, d1.dimensions
            d1.nn, d2.nn = d1.nn.crossover(d2.nn)
        
        return d1, d2
        
    def _createNetwork(self):
        #TODO: random numbers here:
        return neural.randomNeuralNetwork(2, 2, 5)
    
    def clone(self):
        newNN = self.nn.clone()
        return MorphNode(self.dimensions, newNN)
        
        
class MorphConnection(object):
    
    idCounter = 0
    
    HINGE_JOINT = 0
    JOINT_TYPES = [HINGE_JOINT]
    
    _jointValue = MutableChoice(choices=JOINT_TYPES)
    _locationValue = MutableFloat(range=(0, 2*math.pi), rate=0.1)
    _numChannelsValue = MutableInt(range=(1, 3), rate=0.05)
    _recursionLimitValue = MutableInt(range=(1, 4), rate=0.05)
    
    def __init__(self, nodes, joint=HINGE_JOINT, actuators=None, 
                 locations=None, numChannels=None, recursionLimit=1):
        self.id = MorphConnection.idCounter
        MorphConnection.idCounter += 1
        
        self.nodes = nodes
        self.joint = joint
        self.actuators = actuators or self._createActuators()
        self.locations = locations or self._locationValue(repeat=2)
        self.numChannels = numChannels or self._numChannelsValue()
        self.recursionLimit = recursionLimit
        
    def mutate(self):
        self.joint = self._jointValue(self.joint)
        self.locations = self._locationValue(self.locations)
        self.numChannels = self._numChannelsValue(self.numChannels)
        self.recursionLimit = self._recursionLimitValue(self.recursionLimit)
            
        for actuator in self.actuators: actuator.mutate()
        
    def _createActuators(self):
        return Actuator(), Actuator()
    
class Actuator(object):
    
    _strengthValue = MutableFloat(range=(0.0, 1.0))
    
    def __init__(self, strength=None, limits=(0.0, 1.0)):
        self.strength = self._strengthValue()
        self.limits = limits
        
    def mutate(self):
        #TODO: Mutate limits?
        self.strength = self._strengthValue(self.strength)
    
def createBox():
    """Hardcoded test function that creates a simple box creature
    
    (for testing purposes)
    """
    
    box = Morphology()
    box.addNode(MorphNode())

    return box


def createSnake():
    """Hardcoded test function that creates a snake creature
    
    (for testing purposes)
    """
    
    snake = Morphology()
    
    head = MorphNode()
    middle = MorphNode()
    tail = MorphNode()
    
    snake.createConnection(head, middle)
    snake.createConnection(middle, tail)

    return snake

if __name__ == '__main__':
    snake = Morphology()
    
    head = MorphNode()
    middle = MorphNode()
    tail = MorphNode()
    
    snake.createConnection(head, middle)
    snake.addConnection(MorphConnection((middle, middle), recursionLimit=3))
    snake.createConnection(middle, tail)
    
    expanded = snake.expand()
    for node in expanded: print(node)
    print "----"
    s2 = snake.mutate()
    for node in s2.expand(): print(node)
    print "----"
    s3, s4 = snake.crossover(s2)
    for node in s3.expand(): print(s3)
