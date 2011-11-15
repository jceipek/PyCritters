
import operator
import random

class Individual(object):
    
    @property
    def genotype(self): assert False
    
    @property
    def phenotype(self): assert False
        
    def mutate(self):
        assert False
        # return daughter
        
    def cross(self, other):
        assert False
        # return daughter1, daughter2

class Population(object):
    
    def __init__(self, individuals):
        self.individuals = individuals
        
    def calculateFitness(self, fitnessCalculator):
        self.scores = fitnessCalculator.calculate(self.individuals)
        
    @property
    def size(self): 
        return len(self.individuals)
    
    @property
    def averageFitness(self):
        return sum(self.scores.values())/float(self.size)
        
class FitnessCalculator(object):
    
    def calculate(self, individuals):
        assert False
        # return dict mapping from individuals to scores
        
# Fitness are calculated based on individual performance only
class IndividualCompetition(FitnessCalculator):
    
    def calculate(self, individuals):
        return dict((indv, self._doCalculation(indv)) for indv in individuals)
    
    def _doCalculation(self, individual):
        assert False
        # return individual score
        
class RoundRobin(FitnessCalculator):
    
    def calculate(self, individuals):
        scores = {}
        for indv1, index in enumerate(individuals[:-1]):
            for indv2 in individuals[index+1:]:
                pair = indv1, indv2
                
                scores = self._doCalculation(pair)                
                for score, indv in zip(scores, pair):
                    scores[indv] = scores.get(indv, 0) + score
        
        return scores
    
    def _doCalculation(self, pair):
        assert False
        # return 0.0, 0.0
        
class ReproductionMechanism(object):
    
    def __init__(self, createRandomIndividual):
        self.createRandomIndividual = createRandomIndividual
    
    def newPopulation(self, size):
        return Population([self.createRandomIndividual() for _ in range(size)])
    
    def newGeneration(self, population):
        assert False
        
class Spartans(ReproductionMechanism):
    
    def __init__(self, numSpartans, createRandomIndividual):
        ReproductionMechanism.__init__(self, createRandomIndividual)
        self.numSpartans = numSpartans
        
    def _getSpartans(self, scores):
        ranked = sorted(scores.iteritems(), key=operator.itemgetter(1), 
                        reversed=True)
        return ranked[:self.numSpartans]
        
    def newGeneration(self, population):
        spartans = self._getSpartans(population.scores)
        
        newIndividuals = spartans
        for _ in range(population.size - len(newIndividuals)):
            newIndividuals.append(random.choice(spartans).mutate())
        
        return Population(newIndividuals)
        
    
    
    