'''
Created on Nov 11, 2011

@author: wdolphin
'''
import sys
sys.path.append('../')  #add critters to pythonpath to use as library
import time
from visualization import render,renderable
from physics.collisionManager import CollisionManager
from bullet.bullet import DiscreteDynamicsWorld, Vector3, Hinge2Constraint, AxisSweep3, SequentialImpulseConstraintSolver
import pygame

class SimulationEnvironment(object):
    '''
    This object represents a simulation environment, encapsulating a dynamics world and a collisionManager.
    This /should/ include a ground body, but that currently does not work, as multiple imports have different references to Types
    '''
    
    def __init__(self,vis=True):
        '''
        Initializes an Empty Simulation Environment containing only the ground.
        '''
        self.objectList = [] #stores every Physics Object added
        self.cM = CollisionManager()
        self.ents = set()
        worldMin = Vector3(-1000,-1000,-1000) #TODO allow as param
        worldMax = Vector3(1000,1000,1000) #TODO allow as param
        broadphase = AxisSweep3(worldMin, worldMax)
        solver = SequentialImpulseConstraintSolver()
        self.dw = DiscreteDynamicsWorld(None, broadphase, solver)

        
        if vis:
            self.r = render.Renderer(self.dw, debug=True)
            self.r.setup()
        
    def addPhysicsObject(self,physObj,color=None):
        '''
        Adds a PhysicsObject to this SimulationEnvironment, adding it to the collision manager and to the renderables
        '''
        self.objectList.append(physObj)
        self.cM.addPhysicsObject(physObj)
        if color == None:
            color = (255,0,0)
        self.ents.add(renderable.makeRenderable(physObj, color))
    
    def ignoreCollision(self,po1,po2):
        '''
        Selects the two physics objects to have their collisions ignored by the physics engine.
        '''
        self.cM.ignoreCollision(po1,po2)

    def addConstraint(self,constraint):
        '''
        #TODO need to add abstraction here...
        '''
        self.dW.addConstraint(constraint)

    def step(self,rot=0):
        '''
        Simulates one time step in the physic engine, rendering it to the pyGame window
        '''
        timeStep = fixedTimeStep = 1.0 / 600.0
        self.dw.stepSimulation(timeStep, 1, fixedTimeStep)
        now = time.time()
        delay = now % timeStep
        time.sleep(delay)
        self.r.render(self.ents)
        self.r.rotateCamera(rot)
        
    def _run(self):
        '''
        Prepares this SimulationEnvironment to be run, placing all of the
        objects in the DynamicsWorld with their collisions properly mapped. This
        should not be invoked by the user
        '''
        self.cM.calculateCollisionGroups()
        for o in self.objectList:
            print(o)
            self.dw.addRigidBody(o.body,self.cM.getCollisionFilterGroup(o),self.cM.getCollisionFilterMask(o))

            
    def run(self):
        self._run()
        rot = 0
        running = True
        while running:
            self.step(rot)
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
        
        

if __name__ =='__main__':
    print("Not intended to be run as a script")
            
