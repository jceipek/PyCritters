
from __future__ import print_function

import networkx as nx
import copy
import operator
import math
from critters.utils import *
from random import random, choice, sample, randint, normalvariate

def randomNeuralNetwork(numInputs, numOutputs, numUpperLayers, nodesPerLayer):
    assert numUpperLayers >= 1
    
    nn = NeuralNetwork(numInputs, numUpperLayers)
    for layer in nn.upperLayers:
        for _ in range(nodesPerLayer):
            nn.addNode(randomNode(), layer)
            
    nn.connectAll()
    nn.validate(numInputs, numOutputs)
    return nn


class NeuralNetwork(object):
    """A class used to consruct neural networks with multiple layers.
    
    A neural network has (1) an input layer and (2) an upper layer that contains
    inner (hidden) layers (2a).
    
    1) Consists of input nodes that take in a single input from a sensor and
       return an output.
       
    2) Consists of all nodes not in the input layer
    
    ...
    """
    
    INSERT_NODE_MUTATION_RATE = 0.25
    REMOVE_NODE_MUTATION_RATE = 0.2
    EDGE_MUTATION_RATE = 0.1
    CROSSOVER_RATE = 1
    
    def __init__(self, numInputs, numUpperLayers=1):
        self.graph = nx.DiGraph()
        self.layers = [[] for _ in range(numUpperLayers + 1)]
        for _ in range(numInputs): self.addNode(InputNode(), self.inputNodes)
        
    @property
    def nodes(self): return flatten(self.layers)
        
    @property
    def inputNodes(self): return self.layers[0]
    
    @property
    def innerLayers(self): return self.layers[1:-1]
        
    @property
    def outputNodes(self): return self.layers[-1]
    
    @property
    def upperLayers(self): return self.layers[1:]
    
    @property
    def upperNodes(self): return flatten(self.upperLayers)
    
    @property
    def numInputs(self): return len(self.inputNodes)
    
    @property
    def numOutputs(self): return len(self.outputNodes)
    
    def process(self, inputs, dt=1):
        assert len(inputs) == self.numInputs
        
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
    
    def removeNode(self, node, layer=None):
        """Removes a node and its connections from the NN. 
        
        Layer can be specified if it is known what layer the node is in.
        
        NOTE: This may leave the NN in an invalid state where input
        restrictions are not met.
        """
        self.graph.remove_node(node)
        if layer:
            layer.remove(node)
        else:
            for layer in self.layers:
                try:
                    layer.remove(node)
                    break
                except: pass
                
    def addNode(self, node, layer):
        """Adds a node to the graph at the given layer index.
        
        """
        if node not in layer: 
            self.graph.add_node(node)
            layer.append(node)
                
    def connectNode(self, node, layer, forwards=True, backwards=True):
        """Adds and connects node to the graph at the given layer index.
        
        TODO: Document me!
        """
        self.addNode(node, layer)
        layerIndex = self._layerIndex(layer)
        
        if layerIndex > 0 and backwards:
            for prev in self.layers[layerIndex-1]:
                self.makeConnection(prev, node)
        
        if layerIndex < len(self.layers)-1 and forwards:
            for nextNode in self.layers[layerIndex + 1]:
                self.makeConnection(node, nextNode)
        
    
    def makeConnection(self, prev, node, weight=random):
        """Adds a connection outputting from prev to node.
        
        Weight defaults to a random number but can be specified as a function
        or as a scalar value.
        
        Does nothing if the nodes are already connected.
        """
        if self.graph.has_edge(prev, node): return
        
        if callable(weight): weight = weight()
        self.graph.add_edge(prev, node, weight=weight)
        
    def connectAll(self, layers=None, weight=random):
        """Adds connections between all nodes, respecting node requirements.
        
        Weight defaults to a random number but can be specified as a function
        or as a scalar value.
        
        If layers is specified as a list of list of nodes, then only those
        layers are connected, otherwise all layers are connected.
        """
        layers = layers or self.layers
        
        for prevLayer, layer in zip(layers, layers[1:]):
            for prev in prevLayer:
                for node in layer:
                    self.makeConnection(prev, node, weight)
        
    def _validNodeInputs(self, node):
        return len(self.graph.predecessors(node)) >= 1
        
    def _validNodeOutputs(self, node):
        return len(self.graph.successors(node)) >= 1
    
    def _validState(self, numInputs, numOutputs):
        if numInputs != self.numInputs: return False
        if numOutputs != self.numOutputs: return False
        
        for layerIndex, layer in enumerate(self.layers):
            if layerIndex > 0:
                if any(not self._validNodeInputs(node) for node in layer):
                    return False
            if layerIndex < len(self.layers) - 1:
                if any(not self._validNodeOutputs(node) for node in layer):
                    return False
            
        return True
    
    def _nextLayer(self, layer):
        return self.layers[self._layerIndex(layer) + 1]
    
    def _prevLayer(self, layer):
        return self.layers[self._layerIndex(layer) - 1]
    
    def _layerIndex(self, layer):
        return self.layers.index(layer)
    
    def validate(self, numInputs, numOutputs):
        if self._validState(numInputs, numOutputs): return
        
        def fixLayer(layer, diff, newNodeFunc):
            if diff > 0:
                for _ in range(diff): 
                    self.connectNode(newNodeFunc(), layer)
            else:
                for node in sample(layer, abs(diff)): 
                    self.removeNode(node, layer)
                    
        inputDiff = numInputs - self.numInputs
        if inputDiff: fixLayer(self.inputNodes, inputDiff, InputNode)
        
        def buildUpNode(layer, node):
            prevLayer = self._prevLayer(layer)
            if prevLayer:
                for prev in prevLayer: self.makeConnection(prev, node)
            else:
                fixLayer(prevLayer, 1, randomNode)
        
        for layer in self.upperLayers:
            for node in layer:
                if not self.graph.predecessors(node): 
                    buildUpNode(layer, node)
                
            # edge case, we get a layer with no nodes, so we add a node
            # that should always work (any num of inputs).
            if not layer: fixLayer(layer, 1, randomNode)
            
        for layer in self.layers[0:-1]:
            for node in layer:
                if not self.graph.successors(node):
                    self.connectNode(node, layer, backwards=False)
        
        outputDiff = numOutputs - self.numOutputs
        if outputDiff: 
            fixLayer(self.outputNodes, outputDiff, randomNode)
        
        assert self._validState(numInputs, numOutputs)
    
    def mutate(self):
        nn = self.clone()
        
        #mutate nodes
        for node in self.nodes: node.mutate()
        
        if random() < self.INSERT_NODE_MUTATION_RATE:
            node = randomNode()
            nn.connectNode(node, choice(nn.upperLayers))
            
        if random() < self.REMOVE_NODE_MUTATION_RATE:
            nn.removeNode(choice(nn.upperNodes))
        
        for prev, node in self.graph.edges_iter():
            if random() < self.EDGE_MUTATION_RATE:
                current = self.getWeight(prev, node)
                self.setWeight(prev, node, scalarMutate(current))
                
        nn.validate(self.numInputs, self.numOutputs)
        return nn
    
    def crossover(self, other):
        daughters = self.clone(), other.clone()
        
        if random() < self.CROSSOVER_RATE:
            def chooseCrossoverPoint(nn):
                return 1+choice(range(len(self.innerLayers)))
            points = chooseCrossoverPoint(self), chooseCrossoverPoint(other)
            
            
            def subgraph(nn, layers):
                return nn.graph.subgraph(flatten(layers))
            
            def disconnect(nn, layers):
                for layer in layers:
                    for node in layer: 
                        nn.removeNode(node)
            
            def mergeGraph(nn, subgraph):
                nn.graph.add_nodes_from(subgraph.nodes_iter())
                nn.graph.add_edges_from(subgraph.edges_iter(data=True))
                
            def connectLayers(nn, bottom, top):
                nn.connectAll([bottom[-1], top[0]])
                nn.connectAll([top[0], nn.outputNodes])
            
            bottoms = [d.layers[0:p] for d, p in zip(daughters, points)]
            tops = [d.layers[p:-1] for d, p in zip(daughters, points)]
            subgraphs = [subgraph(d, top) for d, top in zip(daughters, tops)]
            
            for d, top in zip(daughters, tops): disconnect(d, top)
            
            mergeGraph(daughters[0], subgraphs[1])
            mergeGraph(daughters[1], subgraphs[0])
            
            connectLayers(daughters[0], bottoms[0], tops[1])
            connectLayers(daughters[1], bottoms[1], tops[0])
            
            daughters[0].layers[points[0]:-1] = tops[1]
            daughters[1].layers[points[1]:-1] = tops[0]
        
        return daughters


def nodeTypes(includeInput=False):
    """Returns a list of Node classes (excluding Node)."""
    types = Node.__subclasses__()
    if not includeInput: 
        types = [t for t in types if t != InputNode]
    return types
    
def randomNodes(n=1, **kwgs):
    """Returns a list with n Node instances.
    
    Selects from all node types except input nodes and initializes them with no
    parameters specified (should yield random parameters).
    """
    types = nodeTypes(**kwgs)
    return [choice(types)() for _ in range(n)]

def randomNode(**kwgs):
    return randomNodes(n=1, **kwgs)[0]
 
class Node(object):
    """The base class for a neural network node.
    
    Nodes that inherit from this class need to override the 'process'
    method. numInputs specifies the total number of inputs a node can
    take. By default, it is unlimited (UNLIMITED_INPUTS).
    
    Attributes:
        output (the output of the node)
    """
    
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
        
    def mutate(self):
        """Has a chance of mutating the parameters of the node (not in place).
        
        This depends on the node type and must thus be overidden by
        all subclasses for which mutation makes sense.
        """
        pass
    
    def _pickInput(self, inputs, floatIndex):
        intIndex = int(round((len(inputs)-1)*floatIndex))
        return inputs[intIndex]

    
class InputNode(Node):
    """A node that returns the input unchanged."""
    
    def process(self, inputs, dt):
        assert len(inputs) == 1
        self.output = inputs[0]
        
    def mutate(self):
        return InputNode()
    
class SumNode(Node):
    """A node that returns the sum of all inputs."""
    
    def process(self, inputs, dt):
        self.output = sum(inputs)
        
class ProductNode(Node):
    """A node that returns the product of all inputs."""
    
    def process(self, inputs, dt):
        self.output = product(inputs)
        
class DivideNode(Node):
    """A node that returns the result of 1/2/.../n for inputs [1,2,...,n]."""
    
    def process(self, inputs, dt):
        self.output = divide(inputs)

class SumThresholdNode(Node):
    """A node that returns 1 if sum(inputs) is greater than threshold.
    
    The threshold attribute is considered to be a parameter, and is thus not
    cleared when clear is called. The threshold may be changed by a mutation.
    """
    
    THRESHOLD_MUTATE_RATE = 0.2
    
    def __init__(self, threshold=None):
        """Threshold defaults to a random number between 0 and 1."""
        if threshold is None: threshold = random()
        self.threshold = threshold
        
    def process(self, inputs, dt):
        self.output = int(sum(inputs) >= self.threshold)
        
    def mutate(self):
        if random() < self.THRESHOLD_MUTATE_RATE:
            self.threshold = scalarMutate(self.threshold)
        
class GreaterThanNode(Node):
    """A node that returns true if i+1>i for i inputs."""
    
    def process(self, inputs, dt):
        self.output = int(all(cur > prev for prev, cur in 
                              zip(inputs, inputs[1:])))
        
class SignOfNode(Node):
    """A node with one input that will return the sign of the input (-1,0,1)."""
    
    INDEX_MUTATION_RATE = 0.2
    
    def __init__(self, index=None):
        """Index defaults to a random number between 0 and 1."""
        if index is None: index = random()
        self.index = index
    
    def process(self, inputs, dt):
        self.output = sign(self._pickInput(inputs, self.index))
        
    def mutate(self):
        if random() < self.INDEX_MUTATION_RATE:
            self.index = clampRange((0, 1), scalarMutate(self.index))

class MinNode(Node):
    """A node that will return its smallest value input."""
    
    def process(self, inputs, dt):
        self.output = min(inputs)
        
class MaxNode(Node):
    """A node that will return its greatest value input."""
    
    def process(self, inputs, dt):
        self.output = max(inputs)
        
class AbsNode(Node):
    """A node with one input that will return the absolute value of an input.
    
    Which input is specified by an index [0,1] where 0 is the first input, 1 is
    the last input and values in between are rounded to the nearest actual 
    index.
    """
    
    INDEX_MUTATION_RATE = 0.2
    
    def __init__(self, index=None):
        """Index defaults to a random number between 0 and 1."""
        if index is None: index = random()
        self.index = index
    
    def process(self, inputs, dt):
        self.output = abs(self._pickInput(inputs, self.index))
        
    def mutate(self):
        if random() < self.INDEX_MUTATION_RATE:
            self.index = clampRange((0, 1), scalarMutate(self.index))
        
class IfNode(Node):
    """A node that will output the first input greater than 0.
    
    If no input is greater than 0, output is 0.
    """
    
    def process(self, inputs, dt):
        for i in inputs:
            if i > 0:
                self.output = i
                return
        
        self.output = 0


if __name__ == '__main__':
    nn1 = randomNeuralNetwork(8, 5, 3, 10)
    nn2 = randomNeuralNetwork(8, 5, 3, 10)
    d1, d2 = nn1.crossover(nn2)
    for _ in range(100): d1 = d1.mutate()
    d1.validate(8, 5)
    print(d1.process(range(8)))

