'''This module provides some basic rendering methods for the physics simulations.

It is intended to be completely separate from the simulation code, but should
be useful for testing and visualization.
'''

import pygame
import pygame.locals

class Renderer(object):

    def __init__(self, world):
        self.world = world
    
    def setup(self):
        pygame.init()
        pygame.display.set_mode((1024, 768))
    
    def render(self, iterableObjects):
        """Draws all of the renderable objects passed in as an iterable.
        
        This uses pygame.
        """
        
        for renderable in iterableObjects:
            renderable.render()

        pygame.display.flip()
