
import networkx as nx
import collections
from random import random, randint, sample, shuffle, choice

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
        connection = createConnection((prev, node))
        _addConnection(graph, connection)
    
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

def crossover(graph1, graph2, rate=0.3, method="graft"):
    if random() > rate: return graph1, graph2
    
    graphs = [graph1, graph2]
    shuffle(graphs)
    
    if method == "graft":
        return _graft(graphs)
    else:
        raise ValueError("No such method: " + method)
    
def _graft(graphs):
    g1, g2 = graphs
    if not g1.nodes() or g2.nodes():
        return g1, g2
    
    root = choice(g2.nodes())
    
    graftNodes, graftConnections = subgraphFromNode(root, g2)
    for node in graftNodes: g1.add_node(node)
    for connection in graftConnections: _addConnection(g1, connection)
    
    replacementSite = choice(g1.nodes())
    incomingConnections = g1.in_edges(replacementSite, data=True)
    g1.remove_node(replacementSite)
    
    for prev, _, data in incomingConnections:
        connection = data['connection'].withNewVertices((prev, root))
        _addConnection(g1, connection)
    
    garbageCollect(g1)
    return g1, g2

def subgraphFromNode(root, graph):
    nodes = set([root])
    connections = set()
    
    queue = collections.deque()
    queue.append(root)
    
    while queue:
        current = queue.popleft()
        for _, neighbor, data in graph.out_edges_iter(current, data=True):        
            connection = data['connection']
            if connection in connections: continue
            
            connections.add(connection)
            
            if neighbor not in nodes:
                nodes.add(neighbor)
                queue.append(neighbor)
                
    return nodes, connections
    
def garbageCollect(graph, shouldIgnore=None):
    shouldIgnore = shouldIgnore or (lambda _: False)
    
    while True:
        toRemove = [node for node in graph.nodes_iter() 
                    if graph.in_degree(node) == 0 and not shouldIgnore(node)]
        if toRemove:
            graph.remove_nodes_from(toRemove)
        else:
            break

def _addConnection(graph, connection):
    prev, node = connection
    graph.add_edge(prev, node, key=connection.id, connection=connection)
    
    
    
    
    
