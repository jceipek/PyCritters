'''
Created on Oct 28, 2011

@author: ckernan
'''

from __future__ import print_function

import networkx as nx
import copy
import operator
import math
import utils
from random import random

def makeNeuralNetwork(numInputs, upperLayers):
    layers = [_makeInputs(numInputs)] + upperLayers
    graph = _makeFeedForwardGraph(layers)
    return NeuralNetwork(layers, graph)
    
def _makeInputs(num):
    return [InputNode() for _ in range(num)]
    
def _makeFeedForwardGraph(layers, weight=1):
    graph = nx.DiGraph()
    for layer in layers: graph.add_nodes_from(layer)
    
    def weights():
        while True:
            if callable(weight): yield weight()
            else: yield weight
    
    def repeat(x): 
        while True: yield x
    
    for prevLayer, layer in zip(layers, layers[1:]):
        for node in prevLayer:
            graph.add_weighted_edges_from(zip(repeat(node), layer, weights()))
            
    return graph

class NeuralNetwork(object):
    """A class used to consruct neural networks with multiple layers.
    
    A neural network has (1) an input layer and (2) an upper layer that contains
    inner (hidden) layers (2a).
    
    1) Consists of input nodes that take in a single input from a sensor and
       return an output.
       
    2) Consists of all nodes not in the input layer
    
    ...
    """
    
    
    def __init__(self, layers, graph):
        self.layers = layers
        self.graph = graph
        
    @property
    def inputNodes(self): return self.layers[0]
    
    @property
    def innerLayers(self):
        return self.layers[1:-1]
        
    @property
    def outputNodes(self): return self.layers[-1]
    
    @property
    def upperLayers(self): return self.layers[1:]
    
    @property
    def numInputs(self): return len(self.inputNodes)
    
    @property
    def numOutputs(self): return len(self.outputNodes)
    
    def process(self, inputs, dt=1):
        for node, inpt in zip(self.inputNodes, inputs):
            node.process([inpt], dt)
        
        for layer in self.upperLayers:
            for node in layer: 
                node.process([pred.output*self.getWeight(pred, node) 
                              for pred in 
                              self.graph.predecessors_iter(node)], dt)
        
        return [node.output for node in self.outputNodes]
    
    def clear(self):
        for node in self.graph.nodes(): node.clear()
    
    def getWeight(self, inputNode, node):
        return self.graph[inputNode][node]['weight']
    
    def setWeight(self, inputNode, node, value):
        self.graph[inputNode][node]['weight'] = value
    
    def randomizeWeights(self, randomFunc=random):
        for node in self.graph.nodes_iter():
            for pred in self.graph.predecessors_iter(node):
                self.setWeight(pred, node, randomFunc())
                
    def clone(self, clear=True):
        newNN = copy.deepcopy(self)
        if clear: newNN.clear()
        return newNN
    
UNLIMITED_INPUTS = -1
 
class Node(object):
    """The base class for a neural network node.
    
    Nodes that inherit from this class need to override the 'process'
    method. numInputs specifies the total number of inputs a node can
    take. By default, it is unlimited (UNLIMITED_INPUTS).
    
    Attributes:
        output (the output of the node)
    """
    
    numInputs = UNLIMITED_INPUTS
    
    def __init__(self):
        self.output = 0
    
    def clear(self):
        """Clears all state associated with a node.
        
        This includes the value of the output attribute and any
        memory attributes defined by the node. Subclasses with
        memory will need to override this method. 
        """
        self.output = 0
    
    def process(self, inputs, dt):
        """Sets the output attribute based on the type of node.
        
        This depends on a list of inputs and a timestep dt (from
        the physics environment). This method needs to be overriden
        by all subclasses.
        """
        pass
    
    def _processReturn(self, inputs, dt):
        """Utility method that returns output after caling process()."""
        self.process(inputs, dt)
        return self.output
        
    def mutate():
        """Has a chance of mutating the parameters of the node.
        
        This depends on the node type and must thus be overidden by
        all subclasses for which mutation makes sense.
        """
        pass
    
class InputNode(Node):
    """A node that returns the input unchanged."""
    numInputs = 1
    
    def process(self, inputs, dt):
        self.output = inputs[0]
    
class SumNode(Node):
    """A node that returns the sum of all inputs."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = sum(inputs)
        
class ProductNode(Node):
    """A node that returns the product of all inputs."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = utils.product(inputs)
        
class DivideNode(Node):
    """A node that returns the result of 1/2/.../n for inputs [1,2,...,n]."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = utils.divide(inputs)

class SumThresholdNode(Node):
    """A node that returns 1 if sum(inputs) is greater than threshold.
    
    The threshold attribute is considered to be a parameter, and is thus not
    cleared when clear is called. The threshold may be changed by a mutation.
    """
    numInputs = UNLIMITED_INPUTS
    
    def __init__(self, threshold=1.0):
        self.threshold = threshold
        
    def process(self, inputs, dt):
        self.output = int(sum(inputs) >= self.threshold)
        
    def mutate():
        self.threshold = utils.scalarMutate(self.threshold)
        
class GreaterThanNode(Node):
    """A node that returns true if i+1>i for i inputs."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = int(all(lambda (prev, cur): cur > prev,
                              zip(inputs, inputs[1:])))
        
class SignOfNode(Node):
    """A node with one input that will return the sign of the input (-1,0,1)."""
    numInputs = 1
    
    def process(self, inputs, dt):
        self.output = utils.sign(inputs[0])

class MinNode(Node):
    """A node that will return its smallest value input."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = min(inputs)
        
class MaxNode(Node):
    """A node that will return its greatest value input."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = max(inputs)
        
class AbsNode(Node):
    """A node with one input that will return the absolute value of the input."""
    numInputs = 1
    
    def process(self, inputs, dt):
        self.output = abs(inputs[0])
        
class IfNode(Node):
    """A node that will return the first input greater than 0."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        for i in inputs:
            if i > 0:
                self.output = i
                return
                

if __name__ == '__main__':
    layers = [[SumNode() for _ in range(3)] for _ in range(3)]
    nn = makeNeuralNetwork(5, layers)
    nn.randomizeWeights()
    inputs = range(5)
    print(nn.clone().process(inputs))

