
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
        self.hinges = None#filled in when buildPhysicsObject() is invoked
        self.rects = None #filled in when buildPhysicsObject() is invoked
        
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
        def createRect(bodyPart):
            return objects.Rect((0.0, 0.0), bodyPart.dimensions, 0.0, 1, 0.5)
        
        def createHinge(connection, r1, r2):
            def positionToLocal(rect, position, negate):
                x = rect.size[0]/2
                y = rect.size[1]/2
                
                if position >= 2: x *= -1
                if position == 0 or position == 3: y *= -1
                
                if negate:
                    return x, y
                else:
                    return -x, -y
            
            return objects.Hinge(
                    r1, positionToLocal(r1, connection.locations[0], False), 
                    r2, positionToLocal(r2, connection.locations[1], True))
        
        root = next(self.morphology.nodes_iter())
        rects = {}
        hinges = []

        for prev, node in nx.bfs_edges(self.morphology, root):
            for part in (prev, node):
                if part not in rects:
                    rects[part] = createRect(part)

            connection = self.morphology.get_edge_data(prev, node)['connection']
            hinge = createHinge(connection, rects[prev], rects[node])
            hinges.append(hinge)
            
            
            prevGlobal = rects[prev].position
            if prev == hinge.physObj1:                
                prevLocal = hinge.local1
                otherLocal = hinge.local2
            else:
                prevLocal = hinge.local2
                otherLocal = hinge.local1
                

            
            otherGlobalx = prevGlobal[0] + prevLocal[0] - otherLocal[0] 
            otherGlobaly = prevGlobal[1] + prevLocal[1] - otherLocal[1]
            
            jointLoc1 = (prevGlobal[0] + prevLocal[0],prevGlobal[1] + prevLocal[1]) #sanity check
            jointLoc2 = (otherGlobalx + otherLocal[0],otherGlobaly+ otherLocal[1]) #calculate relative to both global locs
            
            xDif = abs(jointLoc1[0]-jointLoc2[0])
            yDif = abs(jointLoc1[1]-jointLoc2[1])
            if yDif > 0.1 or xDif > 0.1: #we may need to change this value later
                raise "The joint should be in the same position when calculated from either object..."
                
            rects[node].position = (otherGlobalx,otherGlobaly)
            hinge.globalLoc = jointLoc1

        self.rects = rects.values()
        self.hinges = hinges
            
        return self.rects, self.hinges

