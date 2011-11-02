''' This module provides convenience classes to make physics primatives.

It is intended to be very bare-bones and does not supply these primatives
with mesh, texture, lighting, or even color information.
'''

from bullet.bullet import (
    Vector3, Transform, BoxShape, DefaultMotionState, RigidBody)

class Box(object):
    
    def __init__(self, position, size, mass=2.0, restitution=0.9):
    
        self.size = size
        shape = BoxShape(size/2.0)
        transform = Transform()
        transform.setIdentity()
        transform.setOrigin()
        
        motion = DefaultMotionState()
        motion.setWorldTransform(transform)
        
        self.body = RigidBody(motion, shape, mass)
        self.body.setRestitution(restitution)
        
        self.motion = motion
        

class Plane(object):

    def __init__(self, normalVec, distToOrigin):
        # Vector3d, scalar
        pass #Not yet implemented
