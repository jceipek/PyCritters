'''
Created on Oct 28, 2011

@author: ckernan
'''

from __future__ import print_function

import networkx as nx
import copy
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

class NeuralNetwork:
    
    def __init__(self, layers, graph):
        self.layers = layers
        self.graph = graph
        
    @property
    def inputNodes(self): return self.layers[0]
    
    @property
    def innerLayers(self): return self.layers[1:-1]
        
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
    
class Node:
    
    def __init__(self):
        self.clear()
    
    def clear(self):
        self.output = 0
    
    def process(self, inputs, dt):
        pass
    
class InputNode(Node):
    
    def process(self, inputs, dt):
        self.output = inputs[0]
    
class SumNode(Node):
    
    def process(self, inputs, dt):
        self.output = sum(inputs)

layers = [[SumNode() for _ in range(3)] for _ in range(3)]
nn = makeNeuralNetwork(5, layers)
nn.randomizeWeights()
inputs = range(5)
print(nn.clone().process(inputs))
