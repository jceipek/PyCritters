'''This module provides some basic rendering methods for the physics simulations.

It is intended to be completely separate from the simulation code, but should
be useful for testing and visualization. XXX: Currently, it depends on the physics implementation,
which is PyBox2D. Optimally, it would be implementation independent.
'''

import pygame
import random

class Renderer(object):
    SCREEN_WIDTH = 1024
    SCREEN_HEIGHT = 768

    def __init__(self, world):
        self.world = world
    
    def setup(self, showCoords=False):
        pygame.init()
        self.screen = pygame.display.set_mode((Renderer.SCREEN_WIDTH, Renderer.SCREEN_HEIGHT))
        self.showCoords = showCoords
        if self.showCoords:
            fontName = pygame.font.get_default_font()
            self.font = pygame.font.Font(fontName, 12)
    
    def render(self, offset, PPM):
        """Draws all objects in self.world

        This uses pygame and depends on the physics implementation at the moment.
        offset is the offset used for panning, and PPM is the pixels per meter resolution
        """
        self.screen.fill((0,0,0))
        for body in self.world.bodies: # or: world.bodies
            # The body gives us the position and angle of its shapes
            for fixture in body.fixtures:
                # The fixture holds information like density and friction,
                # and also the shape.
                shape=fixture.shape
                
                # Naively assume that this is a polygon shape. (not good normally!)
                # We take the body's transform and multiply it with each 
                # vertex, and then convert from meters to pixels with the scale
                # factor. 
                realVertices=[(body.transform*v) for v in shape.vertices]

                # But wait! It's upside-down! Pygame and Box2D orient their
                # axes in different ways. Box2D is just like how you learned
                # in high school, with positive x and y directions going
                # right and up. Pygame, on the other hand, increases in the
                # right and downward directions. This means we must flip
                # the y components.
                pgvertices=[(v[0]*PPM+offset[0], Renderer.SCREEN_HEIGHT-v[1]*PPM+offset[1]) for v in realVertices]
                
                c1 = random.randint(100,255)          
                c2 = random.randint(100,255)          
                c3 = random.randint(100,255)          
                color = (c1,c2,c3)

                pygame.draw.polygon(self.screen,color, pgvertices)

                if self.showCoords:
                    for realv,scaledv in zip(realVertices,pgvertices):
                        vstr = '(%.2f,%.2f)' % (realv[0],realv[1])
                        surf = self.font.render(vstr, True, (255,255,255))
                        self.screen.blit(surf, scaledv)

        for jointEdge in body.joints:
            
            ax, ay = jointEdge.joint.anchorA
            ax = int(ax * PPM + offset[0] + 0.5)
            ay = int(Renderer.SCREEN_HEIGHT - (ay * PPM) + offset[1] + 0.5)

            bx, by = jointEdge.joint.anchorB
            bx = int(bx * PPM + offset[0] + 0.5)
            by = int(Renderer.SCREEN_HEIGHT - (by * PPM) + offset[1] + 0.5)


            pygame.draw.circle(self.screen, (0,255,0),(ax,ay), 3)
            pygame.draw.circle(self.screen, (0,0,255), (bx,by), 5, 1)
                    

        pygame.display.flip()
