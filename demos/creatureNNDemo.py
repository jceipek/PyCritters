"""
"""

from critters.physics import simulationEnvironment
from critters.physics import objects

from critters import (neural,life,morph)

morphology = morph.createInchWorm()
net = neural.simpleSineNetwork(1, 1)

creature = life.Critter(numSensors=1,morphology=morphology, neuralNet=net)
simEnv = simulationEnvironment.SimulationEnvironment()

rects,hinges = simEnv.addCreature(creature)

simEnv.simulate(offset=(500,-300))
