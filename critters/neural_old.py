
from __future__ import print_function

import networkx as nx
import copy
import operator
import math
from utils import *
from random import random, choice, sample, randint

def randomNeuralNetwork(numInputs, numOutputs, numUpperLayers, nodesPerLayer):
    assert numUpperLayers >= 1
    
    nn = NeuralNetwork(numInputs, numUpperLayers)
    for layerIndex in range(1, numUpperLayers + 1):
        for _ in range(nodesPerLayer):
            nn.addNode(randomNode(), layerIndex)
            
    nn.connectAll()
    nn.conform(numInputs, numOutputs)
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
    REMOVE_NODE_MUTATION_RATE = 0.1
    
    def __init__(self, numInputs, numUpperLayers=1):
        self.graph = nx.DiGraph()
        self.layers = [[] for _ in range(numUpperLayers + 1)]
        for _ in range(numInputs): self.addNode(InputNode(), 0)
        
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
                try:
                    node.process([pred.output*self.getWeight(pred, node) 
                                  for pred in 
                                  self.graph.predecessors_iter(node)], dt)
                except:
                    print("SHIT")
                    print(str(self.layers.index(layer)) + "/" + str(len(self.layers)))
                    print(layer)
                    print(self.layers[self.layers.index(layer)-1])
                    print(node)
                    print(list(self.graph.predecessors_iter(node)))
                    raise
        
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
                
    def addNode(self, node, layerIndex):
        """Adds a node to the graph at the given layer index.
        
        Layer index of 0 indicates the input layer, upper layers are positive
        integers.
        """
        self.graph.add_node(node)
        self.layers[layerIndex].append(node)
                
    def connectNode(self, node, layerIndex, forwards=True, backwards=True):
        """Adds and connects node to the graph at the given layer index.
        
        Layer index of 0 indicates the input layer, upper layers are positive
        integers.
        
        All nodes on the previous layer are connected as inputs, and all nodes 
        on the next layer are connected as outputs. 
        
        NOTE: This does NOT respect the numInputs restriction of nodes. To 
        fix this, call prune.
        """
        self.graph.add_node(node)
        self.layers[layerIndex].append(node)
        
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
        """
        if callable(weight): weight = weight()
        self.graph.add_edge(prev, node, weight=weight)
        
    def connectAll(self, weight=random):
        """Adds connections between all nodes, respecting node requirements.
        
        Weight defaults to a random number but can be specified as a function
        or as a scalar value.
        """
        for prevLayer, layer in zip(self.layers, self.layers[1:]):
            for prev in prevLayer:
                for node in layer:
                    self.makeConnection(prev, node, weight)
        self.prune()
        
    def _validNodeInputs(self, node):
        numInputs = len(self.graph.predecessors(node))
        if node.numInputs == UNLIMITED_INPUTS: 
            return numInputs >= 1
        else: 
            return numInputs == node.numInputs
        
    def _validNodeOutputs(self, node):
        return len(self.graph.successors(node)) >= 1
    
    def _validState(self, numInputs, numOutputs):
        if numInputs != self.numInputs: return False
        if numOutputs != self.numOutputs: return False
        print("Nums ok...")
        
        for layerIndex, layer in enumerate(self.layers):
            if layerIndex != 0:
                if any(not self._validNodeInputs(node) for node in layer):
                    return False
            if layerIndex != len(self.layers) - 1:
                if any(not self._validNodeOutputs(node) for node in layer):
                    return False
            print("layer %d ok" % layerIndex)
        return True
    
    def _nextLayer(self, layer):
        return self.layers[self._layerIndex(layer) + 1]
    
    def _prevLayer(self, layer):
        return self.layers[self._layerIndex(layer) - 1]
    
    def _layerIndex(self, layer):
        return self.layers.index(layer)
    
    def validate(self, numInputs, numOutputs):
        if self._validState(numInputs, numOutputs): return
        
        def fixLayer(layerIndex, diff, newNodeFunc):
            layer = self.layers[layerIndex]
            if diff > 0:
                for _ in range(diff): 
                    self.connectNode(newNodeFunc(), layerIndex)
            else:
                for node in sample(layer, abs(diff)): 
                    self.removeNode(node, layer)
                    
        inputDiff = numInputs - self.numInputs
        if inputDiff: fixLayer(0, inputDiff, InputNode)
        
        def restrictNode(inputs, node):
            selected = sample(inputs, node.numInputs)
            for inp in filterOut(inputs, selected): 
                self.graph.remove_edge(inp, node)
        
        def buildUpNode(i, inputs, node):
            # i is the index in the upperLayers, so layers[i] is the layer 
            # below it
            if len(self.layers[i]) >= node.minimumRequiredInputs:
                choices = filterOut(self.layers[i], inputs)
                needed = node.minimumRequiredInputs - len(inputs)
                newInputs = sample(choices, needed)
                for prev in newInputs: self.makeConnection(prev, node)
            else:
                self.removeNode(node, self.upperLayers[i])
        
        def doLoop(func):
            for i, layer in enumerate(self.upperLayers):
                for node in list(layer):
                    inputs = self.graph.predecessors(node)
                    func(i, layer, node, inputs)
                    
                # edge case, we get a layer with no nodes, so we add a node
                # that should always work (any num of inputs).
                if not layer: 
                    fixLayer(i+1, 1, lambda: randomNode(includeRestrictedInputs=False))
                assert layer
                    
        def checkTooMany(i, layer, node, inputs):
            if len(inputs) > node.numInputs and not node.unlimitedInputs: 
                restrictNode(inputs, node)
                
        def checkTooFew(i, layer, node, inputs):
            if len(inputs) < node.minimumRequiredInputs:
                buildUpNode(i, inputs, node)
        
        doLoop(checkTooFew)
        doLoop(checkTooMany)
        
        def findOutput(node, layer):
            nextLayer = self._nextLayer(layer)
            
            for match in nextLayer:
                if match.unlimitedInputs:
                    self.makeConnection(node, match)
                    return
            
            i = self._layerIndex(nextLayer)
            self.connectNode(randomNode(includeRestrictedInputs=False), i,
                             backward=True, forward=False)
        
        # check outputs
        for layer in self.innerLayers:
            missingOutput = (n for n in layer if not self.graph.out_edges(n))
            for node in missingOutput: findOutput(node, layer)
        
        outputDiff = numOutputs - self.numOutputs
        if outputDiff: 
            fixLayer(len(self.layers)-1, outputDiff, 
                     lambda: randomNode(includeRestrictedInputs=False))
        
        assert self._validState(numInputs, numOutputs)
    
    def prune(self):
        """Prunes the graph of extraneous/malformed nodes.
        
        Removes extra inputs that a node cannot support and removes nodes that 
        have no outputs.
        
        Should maintain numInputs and numOutputs
        """
        numInputs = self.numInputs
        assert numInputs != 0
        numOutputs = self.numOutputs
        assert numOutputs != 0
        
        def restrictNode(inputs, node):
            selected = sample(inputs, node.numInputs)
            for inp in filterOut(inputs, selected): 
                self.graph.remove_edge(inp, node)
        
        def buildUpNode(i, inputs, node):
            # i is the index in the upperLayers, so layers[i] is the layer 
            # below it
            if len(self.layers[i]) >= node.minimumRequiredInputs:
                choices = filterOut(self.layers[i], inputs)
                needed = node.minimumRequiredInputs - len(inputs)
                newInputs = sample(choices, needed)
                for prev in newInputs: self.makeConnection(prev, node)
            else:
                self.removeNode(node, self.upperLayers[i])
        
        def doLoop(func):
            for i, layer in enumerate(self.upperLayers):
                for node in list(layer):
                    inputs = self.graph.predecessors(node)
                    func(i, layer, node, inputs)
                    
                # edge case, we get a layer with no nodes, so we add a node
                # that should always work (any num of inputs).
                if not layer: 
                    extra = randomNode(includeRestrictedInputs=False)
                    self.connectNode(extra, i + 1)
                assert layer
                    
        def checkTooMany(i, layer, node, inputs):
            if len(inputs) > node.numInputs and not node.unlimitedInputs: 
                restrictNode(inputs, node)
                
        def checkTooFew(i, layer, node, inputs):
            if len(inputs) < node.minimumRequiredInputs:
                buildUpNode(i, inputs, node)
        
        # check outputs
        for layer in self.innerLayers:
            toRemove = (n for n in layer if self.graph.out_edges(n))
            for node in toRemove: self.removeNode(node, layer)
        
        doLoop(checkTooFew)
        doLoop(checkTooMany)
          
        # Edge case, we might not have enough output nodes, so we'll just 
        # add in a few sum nodes.
        #for _ in range(numOutputs - self.numOutputs):
        #    self.connectNode(randomNode(includeRestrictedInputs=False), 
        #                     len(self.layers) - 1)
            
        assert self.numInputs == numInputs
        assert self.numOutputs == numOutputs
            
    def conform(self, numInputs, numOutputs):
        """Ensures that the NN is in a valid state.
        
        Adds and subtracts nodes to make sure that it has the right number of
        inputs and outputs.
        """
        inputDiff = numInputs - self.numInputs
        outputDiff = numOutputs - self.numOutputs
        
        def fixLayer(layerIndex, diff, newNodeFunc):
            layer = self.layers[layerIndex]
            if diff > 0:
                for _ in range(diff): 
                    self.addNode(newNodeFunc(), layerIndex)
            else:
                for node in sample(layer, abs(diff)): 
                    self.removeNode(node, layer)
        
        if inputDiff: 
            fixLayer(0, inputDiff, InputNode)
        if outputDiff: 
            fixLayer(len(self.layers)-1, outputDiff, 
                     lambda: randomNode(includeRestrictedInputs=False))
        
        if inputDiff or outputDiff: self.prune()
        
        assert self.numInputs == numInputs
        assert self.numOutputs == numOutputs
    
    def mutate(self):
        nn = self.clone()
        
        #mutate nodes
        for node in self.nodes: node.mutate()
        
        if random() < self.INSERT_NODE_MUTATION_RATE:
            node = randomNode()
            nn.connectNode(node, randint(1, len(nn.upperLayers)))
            
        if random() < self.REMOVE_NODE_MUTATION_RATE:
            nn.removeNode(choice(nn.upperNodes))
        
        nn.conform(self.numInputs, self.numOutputs)
        return nn
        
    
UNLIMITED_INPUTS = -1

def nodeTypes(includeInput=False, includeRestrictedInputs=True):
    """Returns a list of Node classes (excluding Node)."""
    types = Node.__subclasses__()
    if not includeInput: 
        types = [t for t in types if t != InputNode]
    if not includeRestrictedInputs:
        types = [t for t in types if t.numInputs == UNLIMITED_INPUTS]
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
    
    @property
    def hasLocalMutations(self): return hasattr(self, "mutateLocal")
    
    @property
    def minimumRequiredInputs(self):
        if self.numInputs == UNLIMITED_INPUTS: 
            return 1
        else: 
            return self.numInputs
        
    @property
    def unlimitedInputs(self): return self.numInputs == UNLIMITED_INPUTS
    
    def _processReturn(self, inputs, dt):
        """Utility method that returns output after caling process()."""
        self.process(inputs, dt)
        return self.output
        
    def mutate(self):
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
        
    def mutate(self):
        return InputNode()
    
class SumNode(Node):
    """A node that returns the sum of all inputs."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = sum(inputs)
        
class ProductNode(Node):
    """A node that returns the product of all inputs."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = product(inputs)
        
class DivideNode(Node):
    """A node that returns the result of 1/2/.../n for inputs [1,2,...,n]."""
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = divide(inputs)

class SumThresholdNode(Node):
    """A node that returns 1 if sum(inputs) is greater than threshold.
    
    The threshold attribute is considered to be a parameter, and is thus not
    cleared when clear is called. The threshold may be changed by a mutation.
    """
    numInputs = UNLIMITED_INPUTS
    
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
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        self.output = int(all(cur > prev for prev, cur in 
                              zip(inputs, inputs[1:])))
        
class SignOfNode(Node):
    """A node with one input that will return the sign of the input (-1,0,1)."""
    numInputs = 1
    
    def process(self, inputs, dt):
        self.output = sign(inputs[0])

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
    """A node that will output the first input greater than 0.
    
    If no input is greater than 0, output is 0.
    """
    numInputs = UNLIMITED_INPUTS
    
    def process(self, inputs, dt):
        for i in inputs:
            if i > 0:
                self.output = i
                return
        
        self.output = 0


if __name__ == '__main__':
    nn = randomNeuralNetwork(3, 3, 100, 3)
    print(nn._validState(3, 3))
    nn.validate(3, 3)
    #for _ in range(100):
    #    old = nn
    #    nn = nn.mutate()
    #    print(len(old.nodes), len(nn.nodes))
    #    print(old.numInputs, nn.numInputs)
        
    inputs = range(3)
    print(nn.clone().process(inputs))

