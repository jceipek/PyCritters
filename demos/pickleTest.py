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
from critters import Cdf
from matplotlib import pyplot


def main(main,fil='/home/wdolphin/PyCritters/critters/output/max_4'):
    myCrit = pickle.load(open(fil ,'r'))
    
    cdf = generateCdf(myCrit)
    pyplot.plot(*cdf.Render())
    
    print "SCORE: %.2f" % _doCalculation(myCrit)
    
    
def generateCdf(c, n=20):
    return Cdf.MakeCdfFromList(_doCalculation(c, False) for _ in range(n))

def _doCalculation(individual, vis=True):       
    simEnv = SimulationEnvironment(vis=vis)
    failure = individual, 1e-10
    
    try:
        body = simEnv.addCreature(individual)
    except RuntimeError as e:
        print e
        return failure
    
    if not body: return failure
    
    before = simEnv.getMeanX(body)
    
    def calculateScore():
        score = simEnv.getMeanX(body) - before
        #print "Score %.2f, Before %.2f, MeanX %.2f" % \
        #     (score, before, simEnv.getMeanX(body))
        
        rects, _ = body
        if len(rects) > 8:
            score /= len(rects) - 8
        if simEnv.isBelowGround(body):
            score = 1e-10
        
        #print "Final score: %.2f " % max(self.MIN_FITNESS, score)
        return max(1e-10, score)
    
    def check():
        return calculateScore() >= 4.0
    
    simEnv.simulate(timeToRun=50.0, check=(20.0, check))
    
    finalScore = calculateScore()
    return individual, finalScore

if __name__ == '__main__':
    import sys
    main(*sys.argv)