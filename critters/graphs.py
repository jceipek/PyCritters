
import networkx as nx
from random import random, randint, sample

DEFAULT_MUTATION_RATES = { 'node': 0.3, 
                           'removeConnection': 0.01,
                           'connection': 0.3,
                           'newNodeConnection': 0.01,
                           'addEdgeRatio': 0.1,
                         }

def mutate(graph, createNode, createConnection, rates=DEFAULT_MUTATION_RATES):
    
    def mutateNodes():
        for node in graph.nodes_iter(): 
            if random() < rates['node']: node.mutate()
    
    def mutateConnections():
        toRemove = []
        
        for prev, node, key, data in graph.edges_iter(data=True, keys=True):
            if random() < rates['removeConnection']:
                toRemove.append((prev, node, key))
            elif random() < rates['connection']:
                data['connection'].mutate()
                
        for edge in toRemove: graph.remove_edge(*edge)
        
    def makeEdge(prev, node):
        connection = createConnection(prev, node)
        graph.add_edge(prev, node, key=connection.id, connection=connection)
    
    def insertNewNode():
        newNode = createNode()
        graph.add_node(newNode)
        
        for neighbor in graph.nodes_iter():
            if random() < rates['newNodeConnection']:
                makeEdge(newNode, neighbor)
                
    def createNewConnections():
        nodes = graph.nodes()
        for _ in range(int(len(nodes)*rates['addEdgeRatio'])):
            makeEdge(*sample(nodes, 2))
    
    mutateNodes()
    mutateConnections()
    insertNewNode()
    createNewConnections()

def crossover(graph1, graph2, createConnection, rates=None):
    pass
    
def garbageCollect(graph, shouldIgnore=None):
    shouldIgnore = shouldIgnore or (lambda _: False)
    
    while True:
        toRemove = [node for node in graph.nodes_iter() 
                    if graph.in_degree(node) == 0 and not shouldIgnore(node)]
        if toRemove:
            graph.remove_nodes_from(toRemove)
        else:
            break
    
     

    
    
    
    
    