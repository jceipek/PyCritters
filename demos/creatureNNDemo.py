#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
"""

from critters.physics import simulationEnvironment
from critters.physics import objects

from critters import (neural,life,morph)

#physics.objects.Rect
#critters.objects.Rect


morphology = morph.createInchWorm()
net = neural.simpleSineNetwork(0, 1)
creature = life.Critter(morphology, net, 0)
phenotype = creature.phenotype
rects, hinges = phenotype.buildPhysicsObject()
print rects,hinges

simEnv = simulationEnvironment.SimulationEnvironment(vis=True, gravity=False)
ground = objects.StaticRect(position=(0,1),size=(50,1))

for r in rects:
    print r.position
    simEnv.addPhysicsObject(r)

for h in hinges:
    simEnv.addConstraint(h)

#simEnv.addPhysicsObject(ground)

simEnv.run()
