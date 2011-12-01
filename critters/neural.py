

import networkx as nx
import copy
import math
import graphs
from critters.mutations import *
from critters.utils import *
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

def simpleSineNetwork(numInputs, numOutputs):
    nn = NeuralNetwork()
    
    inputs = [InputNode() for _ in range(numInputs)]
    inner = [SineNode() for _ in range(numOutputs)]
    outputs = [OutputNode() for _ in range(numOutputs)]
    
    for node in (inputs + outputs + inner): nn.addNode(node)
    
    for inpt in inputs: 
        for sine in inner: nn.makeConnection(inpt, sine)
    
    for i in range(numOutputs):
        nn.makeConnection(inner[i], outputs[i])
        
    return nn
    

class NeuralNetwork(object):
    
    DEFAULT_OUTPUT = 0.0
    
    def __init__(self, graph=None):
        self.graph = graph or nx.MultiDiGraph()
        self._hasOutputs = set()
        
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
        
        def registerOutput(node):
            if not node in self._hasOutputs:
                self._hasOutputs.add(node)
        
        for node, inpt in zip(self.inputNodes, inputs):
            node.process([inpt], dt)
            registerOutput(node)
            
        for node in self.upperNodes:
            nodeInputs = [pred.output*self.getWeight(pred, node)
                          for pred in self.graph.predecessors_iter(node)
                          if pred in self._hasOutputs]
            if nodeInputs:
                node.process(nodeInputs, dt)
                registerOutput(node)
        
        outputs = []
        for node in self.outputNodes:
            if node in self._hasOutputs:
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
                for node in sample(list(layer), abs(diff)): 
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
    
    def crossover(self, other):
        daughters = self.clone(), other.clone()
        graphs.crossover(daughters[0].graph, daughters[1].graph, 
                         NeuralConnection)
        return daughters
    
    def clear(self):
        for node in self.nodes: node.clear()
        self._hasOutputs = set()
    
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
    
    _weightValue = MutableFloat(range=(-1.0, 1.0))
    
    def __init__(self, prev, node, weight=None):
        self.prev = prev
        self.node = node
        self.weight = weight or self._weightValue()
        
        self.id = 0 # never allow multiple edges
        
    def mutate(self):
        self.weight = self._weightValue(self.weight)
 
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
    
    _thresholdValue = MutableFloat(range=(0.0, 1.0))
    
    def __init__(self, threshold=None):
        """Threshold defaults to a random number between 0 and 1."""
        Node.__init__(self)
        self.threshold = threshold or self._thresholdValue()
        
    def process(self, inputs, dt):
        self.output = int(sum(inputs) >= self.threshold)
        
    def mutate(self):
        self.threshold = self._thresholdValue(self.threshold)
        
class GreaterThanNode(Node):
    """A node that returns true if i+1>i for i inputs."""
    
    def process(self, inputs, dt):
        self.output = int(all(cur > prev for prev, cur in 
                              zip(inputs, inputs[1:])))
        
class SignOfNode(Node):
    """A node with one input that will return the sign of the input (-1,0,1)."""
    
    _indexValue = MutableFloat(range=(0.0, 1.0), rate=0.1)
    
    def __init__(self, index=None):
        """Index defaults to a random number between 0 and 1."""
        Node.__init__(self)
        self.index = index or self._indexValue()
    
    def process(self, inputs, dt):
        self.output = sign(self._pickInput(inputs, self.index))
        
    def mutate(self):
        self.index = self._indexValue(self.index)

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
    
    _indexValue = MutableFloat(range=(0.0, 1.0), rate=0.1)
    
    def __init__(self, index=None):
        """Index defaults to a random number between 0 and 1."""
        Node.__init__(self)
        self.index = index or random()
    
    def process(self, inputs, dt):
        self.output = abs(self._pickInput(inputs, self.index))
        
    def mutate(self):
        self.index = self._indexValue(self.index)
            
class ConstantNode(Node):
    
    _constantValue = MutableFloat(range=(-4.0, 4.0))
    
    def __init__(self, const=None):
        Node.__init__(self)
        self.const = const or self._constantValue()
        
    def process(self, inputs, dt):
        self.output = self.const
    
    def mutate(self):
        self.const = self._constantValue(self.const)
        
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

class SineNode(Node):
    
    _periodValue = MutableFloat(range=(0.01, 2*math.pi))
    
    def __init__(self, period=None):
        Node.__init__(self)
        self.period = period or self._periodValue()
        self.t = 0
        
    def clear(self):
        Node.clear(self)
        self.t = 0
        
    def process(self, inputs, dt):
        self.t += dt
        self.output = math.sin(self.t/self.period)
        
    def mutate(self):
        self.period = self._periodValue(self.period)
        

if __name__ == '__main__':
    nn = simpleSineNetwork(1, 1)
    for _ in range(10):
        print(nn.process([0], dt=0.5))
    
