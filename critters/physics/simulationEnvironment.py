import sys
sys.path.append('../')  #add critters to pythonpath to use as library
import time
from visualization import render, renderable
import physics.objects
import pygame
from Box2D import b2 
import math

class SimulationEnvironment(object):
    '''
    This object represents a simulation environment, encapsulating a dynamics
    world and a collisionManager.
    '''
    
    def __init__(self, vis=True, gravity=True):
        '''
        Initializes an Empty Simulation Environment containing only the ground.
        '''
        
        self.objectDict = dict() #map from id to physicsObject
                                 # for every PO in this environment 
                                 #TODO: make a property
        self.ents = set()

        #World Creation with gravity
        shouldSleep = True
        if gravity == None:
            self.world = b2.world(gravity=(0,0), doSleep=shouldSleep)
        elif isinstance(gravity, tuple):
            self.world = b2.world(gravity=gravity, doSleep=shouldSleep)
        elif gravity == True:
            self.world = b2.world(gravity=(0,-9.8), doSleep=shouldSleep)
        else:
            self.world = b2.world(gravity=(0,gravity), doSleep=shouldSleep)

        
        self.ground = None #self.world.CreateStaticBody(position=(0,1),
                           #                       shapes=b2.polygonShape(box=(50,2))
                           #                       ) #TODO: This will be defined later on, by other code

        self.vis = vis
        
        if self.vis:
            self.r = render.Renderer(self.world) 
            self.r.setup()
     
    def addPhysicsObject(self, physObj, color=None):
        '''
        Adds a PhysicsObject to this SimulationEnvironment,
        adding it to the collision manager and to the renderables
        '''
        
        if isinstance(physObj, physics.objects.StaticPhysicsObject):
            body = self.world.CreateStaticBody(position=physObj.position,
                                        shapes=b2.polygonShape(box=physObj.size))
        else:
            body = self.world.CreateDynamicBody(position=physObj.position, angle=physObj.angle) 
            body.CreatePolygonFixture(box=physObj.size, density=physObj.density, friction=physObj.friction)

        self.objectDict[physObj.identifier] = body
        
        if color == None:
            color = (255,0,0)
        
        if self.vis:
            self.ents.add(renderable.makeRenderable(physObj, color))
    
    def addHinge(self, physObj1, physObj2, globalLoc):
        '''
        Adds a hinge between physObj1 and physObj2 at globalLoc with degrees of 
        freedom about the vectors unit1 and unit2
        '''

        if not physObj1.identifier in self.objectDict or not physObj2.identifier in self.objectDict:
            raise ValueError('PhysicsObjects must be in the SimulationEnvironment')
        
        b1 = self.objectDict[pysObj1.identifier]
        b2 = self.objectDict[pysObj2.identifier]
        
        joint = self.world.CreateRevoluteJoint(bodyA=b1,
                                               bodyB=b2,
                                               anchor=globalLoc,
                                               lowerAngle = -0.5 * b2.pi,
                                               upperAngle = 0.5 * b2.pi,
                                               enableLimit = True,
                                               maxMotorTorque = 200.0,
                                               motorSpeed = 0,
                                               enableMotor = True)
        


    def ignoreCollision(self,po1,po2):
        '''
        Selects the two physics objects to have their collisions ignored by the physics engine.
        '''
        self.cM.ignoreCollision(po1,po2)

    def addConstraint(self, constraint, po1, po2, globalLoc):
        '''
        #TODO need to add abstraction here...
        '''

        if constraint.joint == MorphConnection.HINGE_JOINT:
            joint = self.addHinge(po1, po2, globalLoc)
        else:
            raise "Unimplemented Constraint Type!"

        # Assumes that there is only one joint between two physics objects
        self.constraintDict[frozenset([po1.identifier,po2.identifier])] = joint
        
    def getConstraint(self,po1,po2):
        '''
        Returns the constraint (if any) found between the two PhysicsObjects po1 and po2.
        If no constraint is found, None is returned
        '''
        try:
            return self.constraintDict[frozenset([po1.identifier,po2.identifier])] 
        except:
            return None

        
    def step(self, offset, ppm, visOnly=False):
        '''
        Simulates one time step in the physic engine, rendering it to the pyGame window.

        offset is a tuple used for panning, and ppm is the size of the physical
        objects on the screen. Note that the physics environment uses meters, while pygame uses pixels.
        '''
        timeStep = fixedTimeStep = 1.0 / 600.0
        
        '''for id1,id2 in self.constraintDict.iterkeys():
            break
            nn1 = self.objectDict[id1].nn
            nn2 = self.objectDict[id2].nn
            inputs1=inputs2=None
            outputs1=nn1.process(inputs1,timeStep)
            outputs2=nn2.process(inputs2,timeStep)
            
            #give them access to the actuators
            #somehow combine their outputs
        '''
        if not visOnly:
            self.world.Step(1.0/60.0, 10, 10) #1/desFPS, velIters, posIters
        self.r.render(self.ents)
        

    def run(self, visOnly=False):
        vOffset = 0
        hOffset = 0
        PPM = 20.0
        running = True

        while running:
            self.step((hOffset, vOffset), PPM, visOnly)

            self.r.render(self.ents)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_LEFT:
                        hOffset -= 0.1
                    elif event.key == pygame.K_RIGHT:
                        hOffset += 0.1
                    elif event.key == pygame.K_UP:
                        vOffset += 0.1
                    elif event.key == pygame.K_DOWN:
                        vOffset -= 0.1
                    elif event.key == pygame.K_PLUS:
                        PPM += 1.0
                    elif event.key == pygame.K_MINUS:
                        PPM -= 1.0


    def simulate(self, timeToRun):
        '''
        TODO: add a simulate function which accepts the amount of time to simulate
        and returns the final state of the environment    
        '''
        pass
        
if __name__ =='__main__':
    print("Not intended to be run as a script")
            
