
import genetics
from utils import flatten, cached
import networkx as nx
from critters.physics import objects

class Critter(genetics.Genotype):
    
    def __init__(self, morphology, neuralNet, numSensors):
        self.morphology = morphology
        self.neuralNet = neuralNet
        self.numSensors = numSensors
    
    @property
    @cached
    def phenotype(self):
        return ReifiedCreature(self.morphology.expand(),
                               self.neuralNet, self.numSensors)
    
class ReifiedCreature(object):
    
    def __init__(self, morphology, neuralNet, numSensors):
        self.morphology = morphology
        self.neuralNet = neuralNet
        self.numSensors = numSensors
        
        self._calculateBodyParts()
        self._calculateConnections()
        self._conformNeuralNet()
        
    def clear(self):
        self.neuralNet.clear()
        
    def think(self, inputs, dt):
        actuatorValues = self.neuralNet.process(inputs, dt)
        return dict(zip(self.actuators, actuatorValues))
        
    def _calculateBodyParts(self):
        self.bodyParts = self.morphology.nodes()
        
    def _calculateConnections(self):
        self.connections = [data['connection'] for _, _, data in 
                            self.morphology.edges_iter(data=True)]
        
        self.actuators = flatten([connection.actuators for connection in
                                  self.connections])
    
    def _conformNeuralNet(self):
        self.neuralNet.conform(self.numSensors, len(self.actuators))
        
    def _getConnections(self, bodyPart):
        return [data['connection'] for _, _, data in 
                self.morphology.edges_iter(bodyPart, data=True)]
                
    def _getAdjacentWithConnection(self, bodyPart):
        for connection in self._getConnections(bodyPart):
            for other in connection.nodes:
                if other is not bodyPart:
                    yield other, connection
                    break
        
    def buildPhysicsObject(self):
        print self.bodyParts, self.connections
        def createRect(bodyPart):
            return objects.Rect((0.0, 0.0), bodyPart.dimensions, 0.0, 1, 0.5)
        
        def createHinge(connection):
            for node in connection.nodes: print node
            r1, r2 = [rects[part] for part in connection.nodes]
            
            def positionToLocal(rect, position):
                x = rect.size[0]/2.0
                y = rect.size[1]/2.0
                
                if position >= 2: x *= -1
                if position == 0 or position == 3: y *= -1
                
                return x, y
            
            return objects.Hinge(r1, positionToLocal(r1, connection.locations[0]), 
                                 r2, positionToLocal(r2, connection.locations[1]))
        
        
        rects = dict((part, createRect(part)) for part in self.bodyParts)    
        hinges = dict((c, createHinge(c)) for c in self.connections)
        
        root = morphNode in rects.iterkeys().next()
        
        def placeRects(root,parent = None):
            for other,connection in self._getAdjacentWithConnection(root):
                if other == parent: #graph is not directional, ensure that we do not repeat... 
                    continue
                rootGlobal = root.position
                hinge = hinges[connection]
                if root == hinge.physObj1:                
                    rootLocal = hinge.local1
                    otherLocal = hinge.local2
                else:
                    rootLocal = hinge.local2
                    otherLocal = hinge.local1
                
                otherGlobalx = rootGlobal[0] + rootLocal[0] - otherLocal[0] 
                otherGlobaly = rootGlobal[1] + rootLocal[1] - otherLocal[1]
                other.position = (otherGlobalx,otherGlobaly)
                hinge.globalLoc = other.position
                placeRects(other,root)
                #place this 'other'
                #place all children of other recursively.
   
        return rects, hinges
    
        
