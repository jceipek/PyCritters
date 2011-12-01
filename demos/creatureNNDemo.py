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
net = neural.simpleSineNetwork(1, 1)
creature = life.Critter(morphology, net, 1)

simEnv = simulationEnvironment.SimulationEnvironment(vis=True, gravity=False)

rects,hinges = simEnv.addCreature(creature)

joint = simEnv.getConstraint(rects[0], rects[1])
print joint.motorSpeed
joint.motorSpeed = 2
#simEnv.addPhysicsObject(ground)

simEnv.run(offset=(500,-300))
