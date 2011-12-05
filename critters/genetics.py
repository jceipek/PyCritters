
import operator
import random
import Cdf
from multiprocessing import Process,Queue,cpu_count,Manager
from Queue import Empty
import math

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
        
    @property
    def maxFitness(self):
        return max(self.scores.values())
        
class FitnessCalculator(object):
    
    def calculate(self, individuals):
        assert False
        # return dict mapping from individuals to scores
        
# Fitness are calculated based on individual performance only
class IndividualCompetition(FitnessCalculator):
    
    def calculate(self, individuals):

        def do_work(q,r,fun): #passed in when creating processes
            while True:
                try:
                    x = q.get(block=False)
                    r.append(fun(x))
                except Empty:
                    break
    
        work_queue = Queue()
        manager = Manager()
        res = manager.list()
        processes = [Process(target=do_work, args=(work_queue,res,self._doCalculation)) for i in range(cpu_count())]
        
        for indv in individuals:
            work_queue.put(indv)
        for p in processes:
            p.start()
        for p in processes:
            p.join()
        print res
        realDict = res.copy()
        print realDict
        assert len(realDict.keys()) == len(individuals)
        return realDict
    
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
        
    def _getTop(self, population, n):
        ranked = sorted(population.scores.iteritems(), 
                        key=operator.itemgetter(1), reverse=True)
        return map(operator.itemgetter(0), ranked[:n])

class Spartans(ReproductionMechanism):
    
    def __init__(self, numSpartans, createRandomIndividual):
        ReproductionMechanism.__init__(self, createRandomIndividual)
        self.numSpartans = numSpartans
        
    def newGeneration(self, population):
        spartans = self._getTop(population, self.numSpartans)
        
        newIndividuals = spartans
        for _ in range(population.size - len(newIndividuals)):
            newIndividuals.append(random.choice(spartans).mutate())
        
        return Population(newIndividuals)
        
class MatedReproduction(ReproductionMechanism):
    
    def __init__(self, createRandomIndividual, 
                 mated=0.5, spartans=0.1, asexual=0.3):
        ReproductionMechanism.__init__(self, createRandomIndividual)
        self.mated = mated
        self.spartans = spartans
        self.asexual = asexual
        
    def newGeneration(self, population):
        newIndividuals = self._getTop(population, 
                                      int(self.spartans*population.size))
        
        fitnessCdf = Cdf.MakeCdfFromDict(population.scores)
        def randomParent(): return fitnessCdf.Random()
        
        for _ in range(int(self.mated*population.size)):
            newIndividuals.append(randomParent().crossover(randomParent()))
        
        for _ in range(int(self.asexual*population.size)):
            newIndividuals.append(randomParent().mutate())
            
        for _ in range(population.size - len(newIndividuals)):
            newIndividuals.append(self.createRandomIndividual())
        
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
            
        
    
    
