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
        
        self.objectDict = dict() #map from id to pyBox2d object
                                 # for every PO in this environment 
                                 #TODO: make a property
                                 
        self.connectionDict = dict() #map from a frozen set of of PhysicsObject ids to a Pybox2d joint
        
        self.creatures = dict() #store a reference to every creature being simulated
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
            self.r.setup(showCoords=True)
        self.physicsStep = 1.0/60.0
        
    def addCreature(self, creature):
        phenotype = creature.phenotype
        rects, hinges = phenotype.buildPhysicsObject()
        for r in rects:
            self.addPhysicsObject(r)

        for h in hinges:
            self.addConstraint(h)

        #TODO: Figure out what this should actually return!
        self.creatures[creature] = phenotype
        return rects,hinges

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
            halfSize = tuple(x/2.0 for x in physObj.size) #box takes half size, not full size
            body.CreatePolygonFixture(box=halfSize, density=physObj.density, friction=physObj.friction)

        
        

        self.objectDict[physObj.identifier] = body
        
    
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
        for phenotype in self.creatures.values():
            actuatorDict  = phenotype.think([0],self.physicsStep)
            #special case because of one actuator in test... need to fixLater
            #neural networks and or actuators need a mapping to the physicsObjects
            #or to the ids at least...
            actuatorValues = actuatorDict.values() #XXX:TODO: preserve order better.
            assert len(actuatorValues) == len(phenotype.hinges) #these must be 1:1, otherwise it makes no sense
            for i in range(len(phenotype.hinges)):
                connection = phenotype.hinges[i]
                joint = self.connectionDict[frozenset([connection.physObj1.identifier,connection.physObj2.identifier])]
                joint.motorSpeed = actuatorValues[i]

        self.world.Step(self.physicsStep, 10, 10) #1/desFPS, velIters, posIters

    def placeGround(self):
        currentMinY = None 
        for body in self.world.bodies: # or: world.bodies
            for fixture in body.fixtures:
                shape=fixture.shape
                
                tMin = min([v[1] for v in  shape.vertices])
                
                if currentMinY == None or tMin < currentMinY:
                    currentMinY = tMin
        self.ground= objects.StaticRect(position=(0,tMin-1),size=(150,1))
        simEnv.addPhysicsObject(self.ground)
        
    def run(self, offset=(0,0),timeToRun=None):
        hOffset, vOffset = offset
        PPM = 20.0
        panningRate = 5
        running = True
        if self.vis:
            clock = pygame.time.Clock()
            pygame.key.set_repeat(1, 5)
        time = 0
        if timeToRun==None:
            shouldRun = lambda t: True
        else:
            shouldRun= lambda t: t < timeToRun
            
        while running and shouldRun(time):
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
                        hOffset += panningRate 
                    elif event.key == pygame.K_RIGHT:
                        hOffset -= panningRate 
                    elif event.key == pygame.K_UP:
                        vOffset += panningRate 
                    elif event.key == pygame.K_DOWN:
                        vOffset -= panningRate 
                    elif event.key == pygame.K_MINUS:
                        PPM -= 1
                    elif event.key == pygame.K_EQUALS:
                        PPM += 1
            time +=self.physicsStep

    def simulate(self,offset=(0,0),timeToRun=10):
        '''
        TODO: add a simulate function which accepts the amount of time to simulate
        and returns the final state of the environment    
        '''
        self.placeGround()
        self.run(offset=offset,timeToRun=timeToRun)
        
if __name__ =='__main__':
    print("Not intended to be run as a script")
            
