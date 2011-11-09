
import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

import time
import visualization.render
from visualization.renderable import makeRenderable
import physics.objects as objects

from bullet.bullet import DiscreteDynamicsWorld, Vector3, Hinge2Constraint, AxisSweep3, SequentialImpulseConstraintSolver

import pygame

def step(world):
    timeStep = fixedTimeStep = 1.0 / 600.0
    world.stepSimulation(timeStep, 1, fixedTimeStep)
    now = time.time()
    delay = now % timeStep
    time.sleep(delay*10)
    
ents = set()
worldMin = Vector3(-1000,-1000,-1000)
worldMax = Vector3(1000,1000,1000)
broadphase = AxisSweep3(worldMin, worldMax)
solver = SequentialImpulseConstraintSolver()

dynamicsWorld = DiscreteDynamicsWorld(None, broadphase, solver)

dynamicsWorld.setGravity(Vector3(0, -9.8, 0)) #turn gravity off

ground = objects.StaticPlane(Vector3(0,1,0), 0.0) #Y is up
dynamicsWorld.addRigidBody(ground.body)


box1 = objects.Box(Vector3(0, 2.5, 0), Vector3(9.0,5.0,5.0))
rBox1 = makeRenderable(box1, (255,0,0))
ents.add(rBox1)

box2 = objects.Box(Vector3(9.0, 2.5, 0), Vector3(9.0,5.0,5.0))
rBox2 = makeRenderable(box2, (255,0,0))
ents.add(rBox2)

dynamicsWorld.addRigidBody(box1.body)
dynamicsWorld.addRigidBody(box2.body)

hinge = Hinge2Constraint(box1.body, box2.body, Vector3(4.5,0,0),Vector3(0,0,1),Vector3(0,1,0))

motors = [hinge.getRotationalLimitMotor(2)]

for motor in motors:
    motor.enableMotor = True
    motor.targetVelocity = 10
    motor.hiLimit = 90
    motor.loLimit = -90
    motor.maxMotorForce = 5
    print(motor.maxMotorForce)
    #print(motor.currentPosition)


dynamicsWorld.addConstraint(hinge)

r = visualization.render.Renderer(dynamicsWorld, debug=True)
r.setup()

running = True
rot = 0.0

import math

magic = 0.0

# ESC to quit; LEFT and RIGHT to change rotation speed.
while running:
    step(dynamicsWorld)
    
    magic += 1.0
    for i in range(len(motors)):
        pass#print(str(i) +":" + str(motors[i].currentPosition))
    #hinge.setStiffness(5,math.sin(magic)*-5000000)
    
    r.render(ents)
    r.rotateCamera(rot)
    
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
                
                
