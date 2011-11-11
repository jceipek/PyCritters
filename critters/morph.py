import networkx as nx
import neural
import math

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
        self.addConnection(MorphConnection(tuple(nodes)))

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
