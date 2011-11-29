
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
        
    def buildPhysicsObject(self):
        def createRect(bodyPart):
            return objects.Rect((0.0, 0.0), bodyPart.dimensions, 0.0, 1, 0.5)
        rects = dict((part, createRect(part)) for part in self.bodyParts)
        
        def createHinge(connection):
            r1, r2 = [rects[part] for part in connection.nodes]
            
            def positionToLocal(rect, position):
                x = rect.size[0]/2.0
                y = rect.size[1]/2.0
                
                if position >= 2: x *= -1
                if position == 0 or position == 3: y *= -1
                
                return x, y
            
            return objects.Hinge(r1, positionToLocal(r1, connection.locations[0]), 
                                 r2, positionToLocal(r2, connection.locations[1]))
            
        hinges = dict((c, createHinge(c)) for c in self.connections)
        
        return rects, hinges
    
        