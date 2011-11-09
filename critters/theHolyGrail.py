'''
Created on Nov 8, 2011

@author: wdolphin
'''

class TheHolyGrail(object):
    '''
    classdocs
    '''
    def __init__(self,params):
        '''
        Constructor
        '''
        self.fitnessFunction = None #function used to determine the fitness of a creature after simulation
        self.simulationLength = 10 #length of time of simulation in virtual seconds
        self.initialPopulationSize = 1 #the number of random initial creatures
        self.creatures = dict() #a dictionary mapping from creatureIDs to creatures and the value of the fitness function 
        self.createInitialPopulation()
    
    def createInitialPopulation(self):
        '''
        Creates initial population, populating the creatures dictionary with id -> creature, fitnessValue mappings
        
        for i in range(self.initialPopulationSize):
            crit = self.createInitialCreature()
            if crit.id in self.creatures:
                continue;
            self.creatures[crit.id] = (crit,None) 
        '''
        pass
    def createInitialCreature(self):
        pass
    
    
    def setupSimulation(self):
        '''
        Setup a single simulation with one (for now) creature
        '''
        pass
    
    def simulateCreatures(self):
        '''
        Iterates through the creatures and simulates each creature
        for key in self.creatures.iterkeys():
            self.simulateCreature(key)
        '''
    pass        
    
    def simulateCreature(self,id):
        '''
        Makes a simulation with the creature of id id, simulates that creature, evaluates the fitness function
        and updates the fitness values in the creatures dictionary
        '''
        pass
    
    def killLeastFit(self):
        '''
        Kills the least fit members of the population by removing them from the creatures dictionary
        '''
        pass
    
    def generateNextPopulation(self):
        '''
        Creates a new creatures dictionary by breeding the most fit creatures from the last generation.
        This may include aSexual reproduction, but no references to the same objects will exist in the new dictionary.
        '''
        pass
    
    