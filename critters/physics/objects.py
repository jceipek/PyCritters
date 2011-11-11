''' This module provides convenience classes to make physics primatives.

It is intended to be very bare-bones and does not supply these primatives
with mesh, texture, lighting, or even color information.
'''

from bullet.bullet import (
    Vector3, Transform,
    DefaultMotionState, RigidBody,
    BoxShape, StaticPlaneShape)

class PhysicsObject(object):
    '''A generic PhysicsObject with a unique readOnly identifier.
    
    This identifier is useful for creating collision groups.'''

    _identifierCount = 0
    print("Sould not execute twice")
    
    def __init__(self):
        self._identifier = PhysicsObject._identifierCount
        PhysicsObject._identifierCount += 1
        print(self, PhysicsObject._identifierCount)

    @property
    def identifier(self): return self._identifier
    
    def __str__(self):
        return str(type(self).__name__) + '_' + str(self.identifier)

class Box(PhysicsObject):
    '''A generic rectangular prism that uses pyBullet's RigidBody.
    
    It has a width, height, and depth specified by the Vector3 size,
    a Vector3 position. The density keyword argument is used only
    when the mass is None, in which case the volume and density determine
    the mass. The restitution coefficient determines how "bouncy" collisions are.
    If it is below 1.0, collisions will be inelastic.'''
    def __init__(self, position, size, mass=None, density=1.0, restitution=0.9):
        PhysicsObject.__init__(self) # Handle unique identification
        self.size = size
        if mass == None:
            mass = density * self.size.x * self.size.y * self.size.z
        shape = BoxShape(size*0.5)
        transform = Transform()
        transform.setIdentity()
        transform.setOrigin(position)
        
        motion = DefaultMotionState()
        motion.setWorldTransform(transform)
        
        self.body = RigidBody(motion, shape, mass)
        self.body.setRestitution(restitution)
        
        self.motion = motion
        

class StaticPlane(PhysicsObject):
    '''A generic plane that extends infinitely in all directions.
    
    It is specified by a Vector3 normalVec and a scalar distance
    to the global origin.'''
    def __init__(self, normalVec, distToOrigin):
        PhysicsObject.__init__(self) # Handle unique identification
        shape = StaticPlaneShape(normalVec, distToOrigin)
        
        self.body = RigidBody(None, shape, 0.0)
