"""
A simple demonstration of how to use the SimulationEnvironment and 
Creatures with an example "Inchworm" creature
"""

from critters.physics import simulationEnvironment
from critters.physics import objects
from critters import (neural,life,morph)
import random
iWorm = morph.Morphology() #instantiate an 'empty' morphology
head = morph.MorphNode(dimensions=(2,2)) #create a node representing the head
tail = morph.MorphNode(dimensions=(2,2)) #create a node representing the tail
conLocations=(random.randint(0,4),random.randint(0,4))#select random vertices
                                                      #to join the body parts at

iWorm.addConnection(morph.MorphConnection((head,tail),locations=conLocations))

net = neural.simpleSineNetwork(1, 1) #create a simple neural network

creature = life.Critter(numSensors=1,morphology=iWorm, neuralNet=net)

simEnv = simulationEnvironment.SimulationEnvironment()

simEnv.addCreature(creature)

simEnv.simulate(offset=(500,-300)) #offset is for initial window placement
