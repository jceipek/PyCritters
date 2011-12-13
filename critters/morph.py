import networkx as nx
from critters import neural
import math
import copy
import graphs

from critters.utils import *
from critters.mutations import *
from random import random, choice

def randomMorphology(numNodes):
    m = Morphology()
    
    nodes = [MorphNode() for _ in range(numNodes)]
    for node in nodes: m.addNode(node)
    
    for prev, node in zip(nodes, nodes[1:]):
        m.createConnection(prev, node)
        
    return m

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
        
        rootCache = set()
        def findRoot(diGraph, current=None):
            try:
                current = current or next(diGraph.nodes_iter())
            except StopIteration:
                return None
            
            pred = [v for v in diGraph.predecessors(current) 
                    if v not in rootCache]
            if not pred: 
                return current
            else: 
                rootCache.add(current)
                return findRoot(diGraph, pred[0])
       
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
                
                #print id(neighbor)
                subroot, subgraph = expandNode(neighbor, subDepth)
                
                graph.add_nodes_from(subgraph.nodes_iter())
                graph.add_edges_from(subgraph.edges_iter(data=True))
                
                connection.nodes = (node, neighbor)
                graph.add_edge(root, subroot, { 'connection': connection })
            
            value = root, graph
            cache[key] = value
            return value
        
        root = findRoot(self.graph)
        return expandNode(root, 0)[1] if root else nx.Graph()
    
    def mutate(self):
        newMorph = self.clone()
        graphs.mutate(newMorph.graph, MorphNode, MorphConnection)
        
        graphs.garbageCollect(newMorph.graph)
        if not newMorph.graph.nodes():
            newMorph.createConnection(MorphNode(), MorphNode())
        
        return newMorph
    
    def crossover(self, other):
        daughters = self.clone(), other.clone()
        graphs.crossover(daughters[0].graph, daughters[1].graph)
        return daughters
    
    def visualize(self):
        nx.draw(self.graph)

class MorphNode(object):
    
    CROSSOVER_RATE = 0.1
    
    _dimensionsValue = MutableFloat(range=(0.1, 5.0), rate=0.05)
    
    def __init__(self, dimensions=None):
        self.dimensions = dimensions or self._dimensionsValue(repeat=2)
        
    @property
    def width(self): return self.dimensions[0]
    
    @property
    def height(self): return self.dimensions[1]
    
    def mutate(self):
        self.dimensions = self._dimensionsValue(self.dimensions)
    
    def clone(self):
        return MorphNode(self.dimensions)
        
        
class MorphConnection(object):
    
    idCounter = 0
    
    HINGE_JOINT = 0
    JOINT_TYPES = [HINGE_JOINT]
    
    _jointValue = MutableChoice(choices=JOINT_TYPES)
    _locationValue = MutableFloat(range=(0.0, 3.9999), rate=0.05)
    _recursionLimitValue = MutableInt(range=(1, 4), rate=0.01)
    
    def __init__(self, nodes, joint=HINGE_JOINT, actuators=None, 
                 locations=None, recursionLimit=1):
        self.id = MorphConnection.idCounter
        MorphConnection.idCounter += 1
        
        self.nodes = nodes
        self.joint = joint
        self.actuators = actuators or self._createActuators()
        self.locations = locations or self._locationValue(repeat=2)
        self.recursionLimit = recursionLimit
        
    def mutate(self):
        self.joint = self._jointValue(self.joint)
        self.locations = self._locationValue(self.locations)
        self.recursionLimit = self._recursionLimitValue(self.recursionLimit)
            
        for actuator in self.actuators: actuator.mutate()
        
    def withNewVertices(self, vertices):
        return MorphConnection(vertices, 
                               self.joint, 
                               copy.deepcopy(self.actuators), 
                               copy.copy(self.locations), 
                               self.recursionLimit)
        
    def _createActuators(self):
        return (Actuator(),)
    
    def __getitem__(self, index):
        return self.nodes[index]
    
class Actuator(object):
    
    _strengthValue = MutableFloat(range=(0.0, 1.0), rate=0.05)
    
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

def createInchWorm(rand=False):
    iWorm = Morphology()
    head = MorphNode(dimensions=(2,2))
    tail = MorphNode(dimensions=(2,2))
    conLocations=(1,3)
    
    if rand:
        conLocations = None
    
    iWorm.addConnection(MorphConnection((head,tail),locations=conLocations))

    return iWorm


if __name__ == '__main__':
    snake = Morphology()
    
    head = MorphNode()
    middle = MorphNode()
    tail = MorphNode()
    
    snake.createConnection(head, middle)
    snake.addConnection(MorphConnection((middle, middle), recursionLimit=3))
    snake.createConnection(middle, tail)
    
    snake.visualize()
    
    expanded = snake.expand()
    nx.draw(expanded)
    
    m1, m2 = snake.crossover(createInchWorm())
    print m1.graph
    print "DONE"
    
