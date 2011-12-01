import time
from critters.visualization.render import Renderer
from critters.physics.objects import StaticPhysicsObject
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
                                 
        self.connectionDict = dict() #map from MorphConnection to a PhysicsConstraint 
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
            self.r = Renderer(self.world) 
            self.r.setup()
     
    def addPhysicsObject(self, physObj, color=None):
        '''
        Adds a PhysicsObject to this SimulationEnvironment,
        adding it to the collision manager and to the renderables
        '''
        print type(physObj)
        if isinstance(physObj, StaticPhysicsObject):
            body = self.world.CreateStaticBody(position=physObj.position,
                                        shapes=b2.polygonShape(box=physObj.size))
        else:
            if len(physObj.size) != 2:
                raise ValueError("Size must be a two-tuple")
            body = self.world.CreateDynamicBody(position=physObj.position, angle=physObj.angle) 
            body.CreatePolygonFixture(box=physObj.size, density=physObj.density, friction=physObj.friction)

        self.objectDict[physObj.identifier] = body
        
        if color == None:
            color = (255,0,0)
        
    
    def _addHinge(self, physObj1, physObj2, globalLoc):
        '''
        Adds a hinge between physObj1 and physObj2 at globalLoc
        '''

        if not physObj1.identifier in self.objectDict or not physObj2.identifier in self.objectDict:
            raise ValueError('PhysicsObjects must be in the SimulationEnvironment')
        
        bod1 = self.objectDict[physObj1.identifier]
        bod2 = self.objectDict[physObj2.identifier]
        
        joint = self.world.CreateRevoluteJoint(bodyA=bod1,
                                               bodyB=bod2,
                                               anchor=globalLoc,
                                               lowerAngle = -0.5 * b2.pi,
                                               upperAngle = 0.5 * b2.pi,
                                               enableLimit = True,
                                               maxMotorTorque = 200.0,
                                               motorSpeed = 0,
                                               enableMotor = True)
        return joint

    def addConstraint(self, physicsConstraint):
        '''
        Given a PhysicsConstraint object, a PyBox2d joint will be created
        and all bookeeping done such that it is properly accounted for in the
        simulation environment.
        The PhysicsConstraint must have a GlobalLoc that is not None.
        This implementation assumes that there is a single joint between a 
        given pair of two PhysicsObjects
        '''
        if not  hasattr(physicsConstraint, 'globalLoc' ) and physicsConstraint.globalLoc !=None:
            raise ValueError("In order to add a constratint to the simenv, it must have a not None globalLoc")
        
        # Note: Assumes that there is only one joint between two physics objects
        
        po1 =physicsConstraint.physObj1
        po2 = physicsConstraint.physObj2
        globalLoc = physicsConstraint.globalLoc
        
        self.connectionDict[frozenset([physicsConstraint.physObj1.identifier,physicsConstraint.physObj2.identifier])] = self._addHinge(po1,po2,globalLoc)
        
        
    def getConstraint(self,po1,po2):
        '''
        Returns the constraint (if any) found between the two PhysicsObjects po1 and po2.
        If no constraint is found, None is returned
        '''
        try:
            return self.connectionDict[frozenset([po1.identifier,po2.identifier])] 
        except:
            return None

        
    def step(self):
        '''
        Simulates one time step in the physic engine, rendering it to the pyGame window.
        objects on the screen. Note that the physics environment uses meters, while pygame uses pixels.
        '''
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
        self.world.Step(1.0/60.0, 10, 10) #1/desFPS, velIters, posIters

    def run(self, visOnly=False):
        vOffset = 0
        hOffset = 0
        PPM = 20.0
        panningRate = 5
        running = True
        if self.vis:
            clock = pygame.time.Clock()
            pygame.key.set_repeat(1, 5)
        while running:
            self.step()
            if self.vis:
                self.r.render((hOffset, vOffset), PPM)
                clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_LEFT:
                        hOffset -= panningRate 
                    elif event.key == pygame.K_RIGHT:
                        hOffset += panningRate 
                    elif event.key == pygame.K_UP:
                        vOffset -= panningRate 
                    elif event.key == pygame.K_DOWN:
                        vOffset += panningRate 
                    elif event.key == pygame.K_MINUS:
                        PPM += 1
                    elif event.key == pygame.K_EQUALS:
                        PPM -= 1


    def simulate(self, timeToRun):
        '''
        TODO: add a simulate function which accepts the amount of time to simulate
        and returns the final state of the environment    
        '''
        pass
        
if __name__ =='__main__':
    print("Not intended to be run as a script")
            
