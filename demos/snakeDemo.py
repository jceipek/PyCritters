"""
"""

from critters.physics import simulationEnvironment
from critters.physics import objects

from critters import (neural,life,morph)

import math


morphology = morph.createSnake()
net = neural.simpleSineNetwork(1, 2)

creature = life.Critter(numSensors=1,morphology=morphology, neuralNet=net)

simEnv = simulationEnvironment.SimulationEnvironment(vis=True, gravity=False)

rects,hinges = simEnv.addCreature(creature)

print [(rect.position, rect.size) for rect in rects]
print hinges

simEnv.run(offset=(500,-300))
