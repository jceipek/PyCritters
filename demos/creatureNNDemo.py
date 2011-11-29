#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
"""

import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

from physics.simulationEnvironment import SimulationEnvironment
from physics import objects

import neural
import life
import morph

morphology = morph.createInchWorm()
net = neural.simpleSineNetwork(0, 1)
creature = life.Critter(morphology, net, 0)
phenotype = creature.phenotype()
rects, hinges = phenotype.buildPhysicsObject()

simEnv = SimulationEnvironment(vis=True, gravity=True)
ground = objects.StaticRect(position=(0,1),size=(50,1))

for r in rects:
    simEnv.addPhysicsObject(r)

for h in hinges:
    simEnv.addHinge(h)

simEnv.addPhysicsObject(ground)

simEnv.run()
