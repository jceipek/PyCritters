
import operator
import random

__all__ = ["sign", "product", "divide", "filterOut", "repeat",
           "flatten", "clampRange", "count"]

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

def divide(numbers, divideByZero=0):
    """Divides an iterable of numbers.
    
    If the iterable is empty, returns 1. If the iterable has 1 element, it 
    returns the reciprocal of the element, otherwise it return the successive
    division of the elements (i.e. 1/a/b/c/d/e for a list [a, b, c, d, e]).
    """
    try:
        return reduce(operator.div, numbers, 1)
    except ZeroDivisionError:
        return divideByZero

def filterOut(base, toRemove):
    return [x for x in base if x not in toRemove]

def repeat(x, call=False): 
    while True: 
        yield x() if call else x
        
def flatten(lst):
    return [x for sublist in lst for x in sublist]

def clampRange(rnge, value, inclusive=False):
    lower, upper = rnge
    if inclusive: upper += 1
    return (value - lower) % (upper - lower) + lower

def count(iterator):
    n = 0
    for _ in iterator: n += 1
    return n
