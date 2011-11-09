

import networkx as nx
import copy
import operator
import math
import graphs
from utils import *
from random import random, choice, sample, randint, normalvariate

def randomNeuralNetwork(numInputs, numOutputs, numInner):
    nn = NeuralNetwork()
    
    inputs = [InputNode() for _ in range(numInputs)]
    inner = randomNodes(numInner)
    outputs = [OutputNode() for _ in range(numOutputs)]
    
    for node in (inputs + outputs + inner): nn.addNode(node)
    
    for node in inputs: 
        nn.connectNode(node, numOutputs=randint(1, numOutputs + numInner))
    for node in inner:
        nn.connectNode(node, numInputs=randint(1, numInputs + numInner - 1), 
                             numOutputs=randint(1, numOutputs + numInner - 1))
    for node in outputs:
        nn.connectNode(node, numInputs=randint(1, numInputs + numInner))
        
    return nn

class NeuralNetwork(object):
    
    DEFAULT_OUTPUT = 0.0
    
    def __init__(self, graph=None):
        self.graph = graph or nx.MultiDiGraph()
        
    @property
    def nodes(self): 
        return self.graph.nodes_iter()
    
    @property
    def inputNodes(self): 
        return (node for node in self.nodes if node.isInput)
    
    @property
    def outputNodes(self):
        return (node for node in self.nodes if node.isOutput)
    
    @property
    def innerNodes(self):
        return (node for node in self.nodes
                if not node.isOutput and not node.isInput)
        
    @property
    def upperNodes(self):
        return (node for node in self.nodes if not node.isInput)
    
    @property
    def lowerNodes(self):
        return (node for node in self.nodes if not node.isOutput)
    
    @property
    def numInputs(self): 
        return count(self.inputNodes)
    
    @property
    def numOutputs(self): 
        return count(self.outputNodes)
    
    def process(self, inputs, dt=1):
        assert len(inputs) == self.numInputs
        
        for node, inpt in zip(self.inputNodes, inputs):
            node.process([inpt], dt)
        hasOutputs = set(self.inputNodes)
            
        for node in self.upperNodes:
            nodeInputs = [pred.output*self.getWeight(pred, node)
                          for pred in self.graph.predecessors_iter(node)
                          if pred in hasOutputs]
            if nodeInputs:
                node.process(nodeInputs, dt)
                hasOutputs.add(node)
        
        outputs = []
        for node in self.outputNodes:
            if node in hasOutputs:
                outputs.append(node.output)
            else:
                outputs.append(self.DEFAULT_OUTPUT)
        return outputs
    
    def removeNode(self, node):
        """Removes a node and its connections from the NN. 
        
        NOTE: This may leave the NN in an invalid state where input
        restrictions are not met.
        """
        self.graph.remove_node(node)
                
    def addNode(self, node):
        """Adds a node to the graph at the given layer index.
        
        """
        self.graph.add_node(node)
        
    def makeConnection(self, prev, node, weight=None):
        connection = NeuralConnection(prev, node, weight)
        self.graph.add_edge(prev, node, key=0, connection=connection)
        
    def connectNode(self, node, numInputs=0, numOutputs=0):
        self.addNode(node)
        
        if numInputs:
            lower = list(self.lowerNodes)
            inputs = sample(lower, min(numInputs, len(lower)))
            for inpt in inputs: self.makeConnection(inpt, node)
        
        if numOutputs:
            upper = list(self.upperNodes)
            outputs = sample(upper, min(numOutputs, len(upper)))
            for output in outputs: self.makeConnection(node, output)
        
    def conform(self, numInputs, numOutputs):
        
        def fixLayer(layer, diff, newNodeFunc, **connectParams):
            if diff > 0:
                for _ in range(diff): 
                    self.connectNode(newNodeFunc(), **connectParams)
            else:
                for node in sample(layer, abs(diff)): 
                    self.removeNode(node)
                    
        inputDiff = numInputs - self.numInputs
        if inputDiff: 
            fixLayer(self.inputNodes, inputDiff, InputNode, numOuputs=5)
        
        outputDiff = numOutputs - self.numOutputs
        if outputDiff: 
            fixLayer(self.outputNodes, outputDiff, randomNode, numInputs=5)
        
        graphs.garbageCollect(self.graph, 
                              shouldIgnore=lambda n: isinstance(n, InputNode))
    
    def mutate(self):
        newNN = self.clone()
        graphs.mutate(newNN.graph, randomNode, NeuralConnection)
        newNN.conform(self.numInputs, self.numOutputs)
        return newNN
    
    def clear(self):
        for node in self.nodes: node.clear()
    
    def getWeight(self, inputNode, node):
        return self.graph[inputNode][node][0]['connection'].weight
    
    def setWeight(self, inputNode, node, value):
        self.graph[inputNode][node][0]['connection'].weight = value
    
    def randomizeWeights(self, randomFunc=random):
        for node in self.graph.nodes_iter():
            for pred in self.graph.predecessors_iter(node):
                self.setWeight(pred, node, randomFunc())
    
    def clone(self, clear=True):
        newNN = copy.deepcopy(self)
        if clear: newNN.clear()
        return newNN
    

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

class NeuralConnection(object):
    
    def __init__(self, prev, node, weight=None):
        self.prev = prev
        self.node = node
        self.weight = weight or random()
        
        self.id = 0 # never allow multiple edges
        
    def mutate(self):
        self.weight = scalarMutate(self.weight)
 
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
    
    isInput = False
    isOutput = False

    
class InputNode(Node):
    """A node that returns the input unchanged."""
    
    def process(self, inputs, dt):
        assert len(inputs) == 1
        self.output = inputs[0]
        
    def mutate(self):
        return InputNode()
    
    isInput = True
    
class SumNode(Node):
    """A node that returns the sum of all inputs."""
    
    def process(self, inputs, dt):
        self.output = sum(inputs)
        
class OutputNode(SumNode):
    
    isOutput = True
        
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
        self.index = index or random()
    
    def process(self, inputs, dt):
        self.output = abs(self._pickInput(inputs, self.index))
        
    def mutate(self):
        if random() < self.INDEX_MUTATION_RATE:
            self.index = clampRange((0, 1), scalarMutate(self.index))
            
class ConstantNode(Node):
    
    CONST_MUTATION_RATE = 0.2
    
    def __init__(self, const=None):
        self.const = const or random()
        
    def process(self, inputs, dt):
        self.output = self.const
    
    def mutate(self):
        if random() < self.CONST_MUTATION_RATE:
            self.const = scalarMutate(self.const)
        
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
    nn = randomNeuralNetwork(3, 2, 10)
    print(count(nn.inputNodes))
    for _ in range(100): nn = nn.mutate()
    print(count(nn.inputNodes))
    print(nn.process(range(3)))