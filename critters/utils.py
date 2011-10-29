
import operator

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