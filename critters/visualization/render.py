'''This module provides some basic rendering methods for the physics simulations.

It is intended to be completely separate from the simulation code, but should
be useful for testing and visualization. XXX: Currently, it depends on the physics implementation,
which is PyBox2D. Optimally, it would be implementation independent.
'''

import pygame

class Renderer(object):
    SCREEN_WIDTH = 1024
    SCREEN_HEIGHT = 768

    def __init__(self, world):
        self.world = world
    
    def setup(self):
        pygame.init()
        pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    
    def render(self, offset, PPM):
        """Draws all objects in self.world

        This uses pygame and depends on the physics implementation at the moment.
        offset is the offset used for panning, and PPM is the pixels per meter resolution
        """
        
        for body in world.bodies: # or: world.bodies
            # The body gives us the position and angle of its shapes
            for fixture in body.fixtures:
                # The fixture holds information like density and friction,
                # and also the shape.
                shape=fixture.shape
                
                # Naively assume that this is a polygon shape. (not good normally!)
                # We take the body's transform and multiply it with each 
                # vertex, and then convert from meters to pixels with the scale
                # factor. 
                vertices=[(body.transform*v)*PPM for v in shape.vertices]

                # But wait! It's upside-down! Pygame and Box2D orient their
                # axes in different ways. Box2D is just like how you learned
                # in high school, with positive x and y directions going
                # right and up. Pygame, on the other hand, increases in the
                # right and downward directions. This means we must flip
                # the y components.
                vertices=[(v[0]+offset[0], SCREEN_HEIGHT-v[1]+offset[1]) for v in vertices]

                pygame.draw.polygon(screen, colors[body.type], vertices)

        pygame.display.flip()
