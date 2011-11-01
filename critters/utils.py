
import operator
import random

def sign(x):
    """Returns the sign of a number.
    
    -1 for negative numbers, 0 for 0, 1 for positive numbers.
    """
    if x < 0: return -1
    if x == 0: return 0
    return 1

def product(numbers):
    """Multiplies an iterable of numbers.
    
    If the iterable is empty, returns 1. If the iterable has 1 element, it 
    returns that one element, otherwise it return the product of all the
    elements.
    """
    return reduce(operator.mul, numbers, 1)

def divide(numbers):
    """Divides an iterable of numbers.
    
    If the iterable is empty, returns 1. If the iterable has 1 element, it 
    returns the reciprocal of the element, otherwise it return the successive
    division of the elements (i.e. 1/a/b/c/d/e for a list [a, b, c, d, e]).
    """
    return reduce(operator.div, numbers, 1)
    
def scalarMutate(aVal, mods=3):
    """Mutates a scalar value using Sims-esque methods.
    
    The value is mutated by adding mods random numbers from a
    Gaussian distribution so that small adjustments are more likely
    than large mutations. The scale of the adjustment is relative to the
    original value (sigma is |original value|^(1/2)).
    """
    #FIXME: The mutations here may be too large
    return aVal + sum(random.normalvariate(0.0, abs(aVal)**0.5) for _ in xrange(mods))
