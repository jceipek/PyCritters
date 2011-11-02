import time

import visualization.render
from visualization.renderable import makeRenderable
import physics.objects as objects

from bullet.bullet import DiscreteDynamicsWorld, Vector3

import pygame

def step(world):
    timeStep = fixedTimeStep = 1.0 / 60.0
    world.stepSimulation(timeStep, 1, fixedTimeStep)
    now = time.time()
    delay = now % timeStep
    time.sleep(delay)
    
ents = set()
dynamicsWorld = DiscreteDynamicsWorld()

ground = objects.StaticPlane(Vector3(0,1,0), 0.0) #Y is up
dynamicsWorld.addRigidBody(ground.body)
for i in range(10):
    box = objects.Box(Vector3(0, 0.5+5*i, 0), Vector3(9.0,5.0,5.0))
    rBox = makeRenderable(box, (255,0,0))
    ents.add(rBox)
    dynamicsWorld.addRigidBody(box.body)

r = visualization.render.Renderer(dynamicsWorld, debug=True)
r.setup()

running = True
rot = 0.0


# ESC to quit; LEFT and RIGHT to change rotation speed.
while running:
    step(dynamicsWorld)
    
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
                
                
