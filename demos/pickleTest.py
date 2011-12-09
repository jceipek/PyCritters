from critters import genetics
from critters.life import ReifiedCreature
from critters import neural
from critters import morph
import networkx as nx
from critters.utils import flatten, cached
from critters.physics import objects
from critters.physics.simulationEnvironment import SimulationEnvironment
from critters.life import Critter
import datetime
import pickle


def main(main,fil='/home/wdolphin/PyCritters/critters/output/max_4'):

    myCrit = pickle.load(open(fil ,'r'))
    
    
    simEnv = SimulationEnvironment()
    
    simEnv.addCreature(myCrit)
    
    simEnv.simulate(offset=(500,-300)) #offset is for initial window placement


if __name__ == '__main__':
    import sys
    main(*sys.argv)