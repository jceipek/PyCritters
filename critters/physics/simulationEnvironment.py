'''
Created on Nov 11, 2011

@author: wdolphin
'''
import sys
sys.path.append('../')  #add critters to pythonpath to use as library
import time
from visualization import render,renderable
from physics.collisionManager import CollisionManager
from physics.objects import StaticPlane
from bullet.bullet import DiscreteDynamicsWorld, Vector3, Hinge2Constraint, AxisSweep3, SequentialImpulseConstraintSolver
import pygame
import math

class SimulationEnvironment(object):
    '''
    This object represents a simulation environment, encapsulating a dynamics
    world and a collisionManager.
    This /should/ include a ground body, but that currently does not work, as multiple imports have different references to Types
    '''
    
    def __init__(self, vis=True):
        '''
        Initializes an Empty Simulation Environment containing only the ground.
        '''
        self.objectList = [] #store every physicsObject in this environment
        self.constraintList = dict() #map from a tuple of ids to a constraint
        self.cM = CollisionManager()
        self.ents = set()
        worldMin = Vector3(-1000,-1000,-1000) #TODO allow as param
        worldMax = Vector3(1000,1000,1000) #TODO allow as param
        broadphase = AxisSweep3(worldMin, worldMax)
        solver = SequentialImpulseConstraintSolver()
        self.dW = DiscreteDynamicsWorld(None, broadphase, solver)
        self.ground =StaticPlane(Vector3(0,1,0), 0.0) #TODO: this may need to be added after items are added to this simev
        self.addPhysicsObject(self.ground)
        
        if vis:
            self.r = render.Renderer(self.dW, debug=True)
            self.r.setup()
     
    def addPhysicsObject(self, physObj,color=None):
        '''
        Adds a PhysicsObject to this SimulationEnvironment, adding it to the collision manager and to the renderables
        '''
        self.objectList.append(physObj)
        self.cM.addPhysicsObject(physObj)
        if color == None:
            color = (255,0,0)
        self.ents.add(renderable.makeRenderable(physObj, color))
    
    def addHinge(self,physObj1,physObj2,globalLoc,unit1=None,unit2=None):
        if not physObj1 in self.objectList or not physObj2 in self.objectList:
            raise ValueError('PhysicsObjects must be in the SimulationEnvironment')
        if unit1 == None:
            unit1 = Vector3(0,0,1)
        if unit2 == None:
            unit2 = Vector3(0,1,0)
        hinge = Hinge2Constraint(physObj1.body, physObj2.body,globalLoc,unit1,unit2)
        self.constraintList[(physObj1.identifier,physObj2.identifier)] = hinge
        self.constraintList[(physObj2.identifier,physObj1.identifier)] = hinge
        self.dW.addConstraint(hinge)
        
    def ignoreCollision(self,po1,po2):
        '''
        Selects the two physics objects to have their collisions ignored by the physics engine.
        '''
        self.cM.ignoreCollision(po1,po2)

    def addConstraint(self,constraint,po1,po2):
        '''
        #TODO need to add abstraction here...
        '''
        self.constraintList[(po1.identifier,po2.identifier)] = constraint
        self.constraintList[(po2.identifier,po1.identifier)] = constraint
        self.dW.addConstraint(constraint)

    def getConstraint(self,po1,po2):
        '''
        Returns the constraint (if any) found between the two PhysicsObjects po1 and po2.
        If the state is somehow inconsistent, i.e. if the ordering of PhysicsObjects matters
        None is returned.
        '''
        val1= self.constraintList[(po1.identifier,po2.identifier)] 
        val2=self.constraintList[(po2.identifier,po1.identifier)]
        if val1 != val2:
            return None #TODO: throw exception or handle mismanaged state
        return val1 
        
    def step(self,rot=0,rotUp=0):
        '''
        Simulates one time step in the physic engine, rendering it to the pyGame window
        '''
        timeStep = fixedTimeStep = 1.0 / 600.0
        self.dW.stepSimulation(timeStep, 1, fixedTimeStep)
        now = time.time()
        delay = now % timeStep
        time.sleep(delay)
        self.r.render(self.ents)
        self.r.rotateCamera(rot,rotUp)
        
    def _run(self):
        '''
        Prepares this SimulationEnvironment to be run, placing all of the
        objects in the DynamicsWorld with their collisions properly mapped. This
        should not be invoked by the user
        '''
        self.cM.calculateCollisionGroups()
        for o in self.objectList:
            self.dW.addRigidBody(o.body,self.cM.getCollisionFilterGroup(o),self.cM.getCollisionFilterMask(o))

            
    def run(self):
        self._run()
        rot = 0
        rotUp=0
        running = True
        while running:
            self.step(rot,rotUp)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_LEFT:
                        rot -= 0.1
                    elif event.key == pygame.K_RIGHT:
                        rot += 0.1
                    elif event.key == pygame.K_UP:
                        rotUp+=0.1
                    elif event.key == pygame.K_DOWN:
                        rotUp-=0.1
    
    def simulate(self, timeToRun):
        '''
        TODO: add a simulate function which accepts the amount of time to simulate
        and returns the final state of the environment    
        '''
        pass
        
if __name__ =='__main__':
    print("Not intended to be run as a script")
            
