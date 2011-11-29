
import operator
import random

class Genotype(object):
    
    @property
    def phenotype(self): 
        assert False
        
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
    def meanFitness(self):
        return sum(self.scores.values())/float(self.size)
    
    def __getitem__(self, index):
        return self.individuals[index]
        
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
        
    def getSpartans(self, scores):
        ranked = sorted(scores.iteritems(), key=operator.itemgetter(1), 
                        reverse=True)
        return map(operator.itemgetter(0), ranked[:self.numSpartans])
        
    def newGeneration(self, population):
        spartans = self.getSpartans(population.scores)
        
        newIndividuals = spartans
        for _ in range(population.size - len(newIndividuals)):
            newIndividuals.append(random.choice(spartans).mutate())
        
        return Population(newIndividuals)
        
class Evolution(object):
    
    def __init__(self, reproductionMechanism, fitnessCalculator, 
                 populationSize=300):
        self.reproductionMechanism = reproductionMechanism
        self.fitnessCalculator = fitnessCalculator
        self.populationSize = populationSize
    
    @property
    def latestGeneration(self): return self.populations[-1]
    
    @property
    def basePopulation(self): return self.populations[0]
    
    @property
    def numGenerations(self): return len(self.populations)
    
    def populate(self):
        self.populations = []
        base = self.reproductionMechanism.newPopulation(self.populationSize)
        base.calculateFitness(self.fitnessCalculator)
        self.populations.append(base)
    
    def run(self, maxSteps=100, onGeneration=None):
        def doGeneration():
            previousGen = self.latestGeneration
            nextGen = self.reproductionMechanism.newGeneration(previousGen)
            nextGen.calculateFitness(self.fitnessCalculator)
            self.populations.append(nextGen)
            
        for _ in range(maxSteps):
            doGeneration()
            if onGeneration and \
                    onGeneration(self.latestGeneration, self.numGenerations):
                break
    
if __name__ == '__main__':
    import mutations
    
    class TestIndividual(Genotype):
        
        _numValue = mutations.MutableFloat(range=(0.0, 1.0), stdDev=0.01)
        
        def __init__(self, num=None):
            self.num = num or self._numValue()
        
        @property
        def phenotype(self): return 5*self.num
        
        def mutate(self): 
            return TestIndividual(self._numValue(self.num))
        
        def crossover(self, other):
            avg = (self.num + other.num) / 2.0
            return TestIndividual(avg), TestIndividual(avg)
        
    class TestCompetition(IndividualCompetition):
        
        def __init__(self, target=2.5):
            self.target = target
            
        def _doCalculation(self, individual):
            return self.target**2 - (individual.phenotype - self.target)**2
        
    evo = Evolution(Spartans(5, TestIndividual), TestCompetition(), 10)
    evo.populate()
    evo.run(10)
    print evo.basePopulation.meanFitness, evo.latestGeneration.meanFitness
    print evo.basePopulation[0].num, evo.latestGeneration[0].num
            
        
    
    