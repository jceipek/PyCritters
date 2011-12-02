"""
"""

from critters.physics import simulationEnvironment
from critters.physics import objects

from critters import (neural,life,morph)

import math

#physics.objects.Rect
#critters.objects.Rect


morphology = morph.createSnake()
net = neural.simpleSineNetwork(1, 2)
creature = life.Critter(morphology, net, 1)

simEnv = simulationEnvironment.SimulationEnvironment(vis=True, gravity=False)

#r1 = objects.Rect((0, 0), angle=0.0)
#simEnv.addPhysicsObject(r1)
#r2 = objects.Rect((1, 1), angle=0.0)
#simEnv.addPhysicsObject(r2)
#r3 = objects.Rect((2, 0), angle=0.0)
#simEnv.addPhysicsObject(r3)
#
#j1 = simEnv._addHinge(r1, r2, (0.5, 0.5))
#j2 = simEnv._addHinge(r2, r3, (1.5, 0.5))
#
#j1.motorSpeed = -0.5
#j2.motorSpeed = 1

rects,hinges = simEnv.addCreature(creature)

print [(rect.position, rect.size) for rect in rects]
print hinges

#joint = simEnv.getConstraint(rects[0], rects[2])
#print joint.motorSpeed
#joint.motorSpeed = 2

#simEnv.addPhysicsObject(ground)

simEnv.run(offset=(500,-300))
