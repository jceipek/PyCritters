
import random
from utils import clampRange

__all__ = ["MutableInt", "MutableFloat", "MutableChoice"]

class MutableValue(object):
    
    def __init__(self, rate):
        self.rate = rate
    
    def mutate(self, x):
        if random.random() < self.rate:
            return self._doMutate(x)
        else:
            return x
        
    def mutateAll(self, xs):
        return [self.mutate(x) for x in xs]
    
    def _doMutate(self, x):
        assert False
        
    def random(self, repeat=None):
        if repeat:
            return [self._doRandom() for _ in range(repeat)]
        else:
            return self._doRandom()
        
    def _doRandom(self):
        assert False
    
    def __call__(self, *args, **kwargs):
        if args:
            assert len(args) == 1
            x = args[0]
            
            try:
                iter(x)
                return self.mutateAll(x)
            except TypeError:
                return self.mutate(x)
        else:
            return self.random(**kwargs)
    
class MutableInt(MutableValue):
    
    def __init__(self, range, rate=0.3, stdDev=1.0):
        MutableValue.__init__(self, rate)
        self.range = range
        self.stdDev = stdDev
        
    def _doMutate(self, x):
        x += round(random.normalvariate(0.0, self.stdDev))
        x = clampRange(self.range, x, inclusive=True)
        return int(x)
    
    def _doRandom(self):
        a, b = self.range
        return random.randint(a, b)
    
class MutableFloat(MutableValue):
    
    def __init__(self, range=(0.0, 1.0), rate=0.3, stdDev=0.1):
        MutableValue.__init__(self, rate)
        self.range = range
        self.stdDev = stdDev
        
    def _doMutate(self, x):
        x += random.normalvariate(0.0, self.stdDev)
        x = clampRange(self.range, x)
        return x
    
    def _doRandom(self):
        a, b = self.range
        return a + random.random()*(b - a)

class MutableChoice(MutableValue):
    
    def __init__(self, choices, rate=0.3):
        MutableValue.__init__(self, rate)
        self.choices = choices
        
    def _doMutate(self, x):
        return self._doRandom()
    
    def _doRandom(self):
        return random.choice(self.choices)
    
