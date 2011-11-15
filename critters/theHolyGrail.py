
'''
Created on Nov 8, 2011

@author: wdolphin
'''

import math

from physics.simulationEnvironment import SimulationEnvironment
from physics.objects import Box, PhysicsObject
from bullet.bullet import Vector3, Quaternion
from morph import MorphNode

class TheHolyGrail(object):
    '''
    classdocs
    '''
    def __init__(self,params=None):
        '''
        Constructor
        '''
        self.fitnessFunction = None #function used to determine the fitness of a creature after simulation
        self.simulationLength = 10 #length of time of simulation in virtual seconds
        self.initialPopulationSize = 1 #the number of random initial creatures
        self.creatures = dict() #a dictionary mapping from creatureIDs to creatures and the value of the fitness function 
        self.createInitialPopulation()
    
    def createInitialPopulation(self):
        '''
        Creates initial population, populating the creatures dictionary with id -> creature, fitnessValue mappings
        
        for i in range(self.initialPopulationSize):
            crit = self.createInitialCreature()
            if crit.id in self.creatures:
                continue;
            self.creatures[crit.id] = (crit,None) 
        '''
        pass
    def createInitialCreature(self):
        pass
    
    
    def setupSimulation(self):
        '''
        Setup a single simulation with one (for now) creature
        '''
        pass
    
    def simulateCreatures(self):
        '''
        Iterates through the creatures and simulates each creature
        for key in self.creatures.iterkeys():
            self.simulateCreature(key)
        '''
    pass
    
    def simulateCreature(self,id):
        '''
        Makes a simulation with the creature of id id, simulates that creature,
        evaluates the fitness function and updates the fitness values in the
        creatures dictionary
        '''
        pass
    
    def killLeastFit(self):
        '''
        Kills the least fit members of the opulation by removing them from the creatures dictionary
        '''
        pass
    
    def generateNextPopulation(self):
        '''
        Creates a new creatures dictionary by breeding the most fit creatures
        from the last generation. This may include aSexual reproduction, but no
        references to the same objects will exist in the new dictionary.
        '''
        pass
    
    def processMorphologyTree(self,morphology,simEnv=None):
        '''
        Creates the physics engine's representation of this morphology tree. If
        a simulation environment is passed in, this environment is used,
        otherwise one is created. A reference to this simulation environment is
        returned for later simulation
        '''

        if simEnv == None:
            simEnv = SimulationEnvironment()

        graph = morphology.graph
        nodeList = graph.nodes()
        start = nodeList[0] #we can begin at any arbitrary node.
        startPhys = self._makePhysicsObjectFromNode(start,Vector3(0,0,0)) #place first object at the origin
        simEnv.addPhysicsObject(startPhys)
        for node1 in graph.neighbors(start):
            self._addNextPhysicsObject(startPhys,start,node1,graph,simEnv)

        return simEnv

    
    def _addNextPhysicsObject(self,node1phys, node1,node2,graph,simEnv):
        '''
        @precondition node1 must be in the simEnv
        Adds node2 into the physics world with position determined by connection,
        holding the position of node1 constant.
        This function assumes there are no cycles in the graph.
        '''
        #find connection c between n1 n2
        #compute the position of n2 given the position of n1 and the connection c
        #put n2 into simEnv
        #create a hinge between n1 and n2
        #add the hinge between n1 and n2
        #TODO manage motor in hinge
        #add n1 and n2 to ignore collisions between eachother
        c = graph.get_edge_data(node1,node2)[0]['connection']
        self._placeWithRespectTo(node1phys,node2,c,simEnv)
        '''
        for node3 in graph.neighbors(node2):
            if node3 != node1: 
                self._addNextPhysicsObject(self,node2,node3,graph,simEnv)
        '''
    def _placeWithRespectTo(self,placedPhysicsObject, secondMorphNode, connection, simEnv):
        rootGlobalVector = placedPhysicsObject.motion.getWorldTransform().getOrigin()
        rootLocalConnectionVector = rootGlobalVector + self._getVector3FromValue(placedPhysicsObject, connection.locations[0])
        
        #Connection point
        globalConnectionVector = rootGlobalVector + rootLocalConnectionVector 
        angleOfMapping = math.acos(globalConnectionVector.dot(rootGlobalVector))
        newGlobalVector = globalConnectionVector + self._getVector3FromValue(secondMorphNode, connection.locations[1])
        
        axisOfRotation = globalConnectionVector.cross(rootGlobalVector)

        rotationQuat = Quaternion.fromAxisAngle(axisOfRotation, angleOfMapping)

        newPhysObj = self._makePhysicsObjectFromNode(secondMorphNode, pos=newGlobalVector)
        
        simEnv.addPhysicsObject(newPhysObj)
        simEnv.ignoreCollision(placedPhysicsObject,newPhysObj)


    def _getVector3FromValue(self, node, value):
        '''
        @params value: an integer  value for 0 to 64
        @retrun vector3: a vector3 representing the point in local space that
        this integer maps to
        '''
        modded = value % 64
        val1 = modded//16
        
        modded = modded - (val1*16)
        val2= modded//4
        
        modded=modded-(val2*4)
        val3 = modded
        
        w,h,d = (1,1,1)
        if isinstance(node, MorphNode):
           w,h,d = node.width, node.height, node.depth 
        else :
            if isinstance(node, PhysicsObject):
                dims = node.size
                w,h,d = dims.x, dims.y, dims.z
        
        translationalVector = Vector3(w/2.0,h/2.0,d/2.0)
        #theunshifted vector only provides values from 0 to (width,height,depth)
        #we must shift this vector to the corner of the box to have it actually
        #represent all points in local space
        unshifted = Vector3(val1*w,val2*h,val3*d)
        return  unshifted + translationalVector

    def _makePhysicsObjectFromNode(self,aNode,pos=None):
        '''
        Creates a Physics Object from a given morphNode. If position is None,
        default is the origin
        '''
        if pos == None:
            pos = Vector3(0,0,0)
        return Box(pos,Vector3(aNode.width,aNode.height,aNode.depth))
        
    def _processMorphologyTree(self,root):
        pass

if __name__ == '__main__':
    import morph
    iWorm = morph.createInchWorm()
    grail = TheHolyGrail()
    simEnv = grail.processMorphologyTree(iWorm)
    for val in simEnv.objectList:
        print(val)
    simEnv.run()

