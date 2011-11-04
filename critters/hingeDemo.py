import time

import visualization.render
from visualization.renderable import makeRenderable
import physics.objects as objects

from bullet.bullet import DiscreteDynamicsWorld, Vector3, Hinge2Constraint, AxisSweep3, SequentialImpulseConstraintSolver

import pygame

def step(world):
    timeStep = fixedTimeStep = 1.0 / 60.0
    world.stepSimulation(timeStep, 1, fixedTimeStep)
    now = time.time()
    delay = now % timeStep
    time.sleep(delay)
    
ents = set()
worldMin = Vector3(-1000,-1000,-1000)
worldMax = Vector3(1000,1000,1000)
broadphase = AxisSweep3(worldMin, worldMax)
solver = SequentialImpulseConstraintSolver()

dynamicsWorld = DiscreteDynamicsWorld(None, broadphase, solver)

ground = objects.StaticPlane(Vector3(0,1,0), 0.0) #Y is up
dynamicsWorld.addRigidBody(ground.body)

box1 = objects.Box(Vector3(0, 10, 0), Vector3(9.0,5.0,5.0))
rBox1 = makeRenderable(box1, (255,0,0))
ents.add(rBox1)

box2 = objects.Box(Vector3(9.0, 15, 0), Vector3(9.0,5.0,5.0))
rBox2 = makeRenderable(box2, (255,0,0))
ents.add(rBox2)

dynamicsWorld.addRigidBody(box1.body)
dynamicsWorld.addRigidBody(box2.body)

hinge = Hinge2Constraint(box1.body, box2.body, Vector3(4.5,12.5,0),Vector3(0,0,1),Vector3(0,1,0))

hinge.enableSpring(5,1)


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
    hinge.setStiffness(5,math.sin(magic)*-5000)
    
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
                
                
