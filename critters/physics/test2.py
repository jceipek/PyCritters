import pygame.locals
import pygame.display
import time

from OpenGL.GL import (
    GL_DEPTH_TEST, GL_LINES, glEnable, glTranslate,GL_TRIANGLE_STRIP, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
    glPushMatrix, glPopMatrix, glColor, glClear,
    glBegin, glEnd, glTranslate, glRotate, glVertex)
from OpenGL.GLU import gluPerspective,gluNewQuadric, gluSphere
from OpenGL.GLUT import glutSolidCube, glutInit

from bullet.bullet import (
    DRAW_WIREFRAME, DRAW_AABB, DRAW_CONTACT_POINTS, Vector3, Transform, BoxShape, SphereShape, DefaultMotionState, RigidBody,DiscreteDynamicsWorld)

import objects

def step(world):
    timeStep = fixedTimeStep = 1.0 / 60.0
    world.stepSimulation(timeStep, 1, fixedTimeStep)
    now = time.time()
    delay = now % timeStep
    time.sleep(delay)


def render(ents):
    glPushMatrix()
    for o in ents:
        if o.__class__ == objects.Box:
            glPushMatrix()
            a = o.motion.getWorldTransform().getOrigin()
            x = o.motion.getWorldTransform().getRotation()
            glColor(*(255, 0, 0))
            glTranslate(a.x, a.y, a.z)
            glRotate(x.getAngle()*57.2957795,x.getAxis().x,x.getAxis().y,x.getAxis().z)
            glutSolidCube(o.size.x)
            glPopMatrix()
    glPopMatrix()

    pygame.display.flip()
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


def simulate(world, ents, debug):
    while True:
        step(world)
        
        debug.reset()
        world.debugDrawWorld()
        glBegin(GL_LINES)
        for line in debug.lines:
            glColor(*line[6:])
            glVertex(*line[:3])
            glVertex(*line[3:6])
        if debug.contacts:
            pass #print 'Contact!', debug.contacts
        glEnd()
        
        render(ents)

class DebugDraw:
    mode = DRAW_WIREFRAME | DRAW_AABB | DRAW_CONTACT_POINTS

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

def setup():
    pygame.init()
    glutInit()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

def main():
    setup()
    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)

    ents = []
    dynamicsWorld = DiscreteDynamicsWorld()
    
    debug = DebugDraw()
    dynamicsWorld.setDebugDrawer(debug)
    
    ents.append(objects.StaticPlane(Vector3(0,1,0), 0.0)) # Y is up
    ents.append(objects.Box(Vector3(0, 40, 1), Vector3(5.0,5.0,5.0)))

    for o in ents:
        dynamicsWorld.addRigidBody(o.body)

    simulate(dynamicsWorld, ents, debug)
    
if __name__ == '__main__':
    main()
