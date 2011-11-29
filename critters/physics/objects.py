''' This module provides convenience classes to make physics primatives.

It is intended to be very bare-bones and does not supply these primatives
with mesh, texture, lighting, or even color information.
'''

class PhysicsObject(object):
    '''A generic PhysicsObject with a unique readOnly identifier.
    
    This identifier is useful for creating collision groups.'''
    

    _identifierCount = 0    

    def __init__(self):
        self._identifier = PhysicsObject._identifierCount
        PhysicsObject._identifierCount += 1

    @property
    def identifier(self): return self._identifier
    
    def __str__(self):
        return str(type(self).__name__) + '_' + str(self.identifier)

class StaticPhysicsObject(PhysicsObject):
    pass

class DynamicPhysicsObject(PhysicsObject):
    pass


class Rect(DynamicPhysicsObject):
    '''A generic rectangular prism that uses Box2D's CreatePolygonFixture.
    It takes in a Box2D world, position and size tuples, and angle,
    density, and friction floats.
    ''' 
    def __init__(self, position, size=(1,1), angle=0.0, density=1.0, friction=0.3):
        PhysicsObject.__init__(self) # Handle unique identification
        self.size = size
        self.position = position
        self.angle = angle
        self.density = density
        self.friction = friction

class StaticRect(StaticPhysicsObject):
    '''A generic plane that extends infinitely in all directions.
    It takes in a Box2D world and position and size tuples.'''
    def __init__(self, position, size):
        PhysicsObject.__init__(self) # Handle unique identification
        
        self.size = size
        self.position = position
