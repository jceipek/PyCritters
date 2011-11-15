'''This module provides some basic rendering methods for the physics simulations.

It is intended to be completely separate from the simulation code, but should
be useful for testing and visualization. Note that this module uses deprecated
OpenGL methods and shold not be used as a reference for how to write proper
visualization code. Since graphics cards still support these outdated methods
and we want to focus on the algorithms and simulations, we consider this to
be an acceptable tradeoff. In the future, we may replace this module with a
cleaner implementation that uses modern methods.
'''

import pygame
import pygame.locals

from OpenGL.GL import (
    GL_DEPTH_TEST, 
    GL_LINES, 
    GL_COLOR_BUFFER_BIT,  
    GL_DEPTH_BUFFER_BIT,
    GL_LINE_SMOOTH,
    glEnable, 
    glPushMatrix,
    glPopMatrix,
    glClear,
    glBegin, 
    glEnd, 
    glTranslate, 
    glRotate,
    glVertex,
    glColor)

from bullet.bullet import (DRAW_WIREFRAME, DRAW_AABB, DRAW_CONTACT_POINTS)

from OpenGL.GLU import gluPerspective
from OpenGL.GLUT import glutInit

class Renderer(object):

    def __init__(self, world, debug=False):
        self.world = world
        self.cameraAngle = 0.0
        self.debug = debug
        if self.debug:
            self.debugDrawer = DebugDrawer()
            self.world.setDebugDrawer(self.debugDrawer)
        else:
            self.debugDrawer = None
    
    def setup(self):
        pygame.init()
        glutInit()
        pygame.display.set_mode((1024, 768),
                                pygame.locals.OPENGL |
                                pygame.locals.DOUBLEBUF)
        glEnable(GL_DEPTH_TEST|GL_LINE_SMOOTH)
        gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
        glTranslate(0, -4, -60)
    
    def render(self, iterableObjects):
        """Draws all of the renderable objects passed in as an iterable.
        
        This uses pygame and deprecated OpenGL methods.
        """
        
        if self.debug:
            self.debugDraw()
        
        for renderable in iterableObjects:
            renderable.render()
            
        glRotate(self.cameraAngle,0,1,0)
            
        pygame.display.flip()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
         
    def rotateCamera(self, angle):
        self.cameraAngle = angle
            
    def debugDraw(self):
        self.debugDrawer.reset()
        self.world.debugDrawWorld()
        glBegin(GL_LINES)
        for line in self.debugDrawer.lines:
            glColor(*line[6:])
            glVertex(*line[:3])
            glVertex(*line[3:6])
        if self.debugDrawer.contacts:
            pass #print 'Contact!', debug.contacts
        glEnd()

            
            
class DebugDrawer:
    mode = DRAW_WIREFRAME #| DRAW_AABB | DRAW_CONTACT_POINTS

    def reset(self):
        self.lines = []
        self.contacts = []


    def drawLine(self, *args):
        self.lines.append(args)


    def drawContactPoint(self, *args):
        self.contacts.append(args)


    def setDebugMode(self, mode):
        self.mode = mode


    def getDebugMode(self):
        return self.mode
