# Copyright (c) PyBullet Team
# See LICENSE for details.

import time
import pygame.locals, pygame.display

from OpenGL.GL import (
    GL_DEPTH_TEST, GL_LINES, glEnable, glTranslate,GL_TRIANGLE_STRIP, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
    glPushMatrix, glPopMatrix, glColor, glClear,
    glBegin, glEnd, glTranslate, glRotate, glVertex)
from OpenGL.GLU import gluPerspective,gluNewQuadric, gluSphere
from OpenGL.GLUT import glutSolidCube, glutInit

from bullet.bullet import (
    DRAW_WIREFRAME, DRAW_AABB, DRAW_CONTACT_POINTS, Vector3, Transform, BoxShape, SphereShape, DefaultMotionState, RigidBody,DiscreteDynamicsWorld)

def main():
    pygame.init()
    glutInit()
    pygame.display.set_mode(
        (1024, 768), pygame.locals.OPENGL | pygame.locals.DOUBLEBUF)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60.0, 640.0 / 480.0, 0.5, 1000.0)
    glTranslate(0, -15, -60)

    objects = []
    dynamicsWorld = DiscreteDynamicsWorld()
    
    debug = DebugDraw()
    dynamicsWorld.setDebugDrawer(debug)

    objects.append(Ground())
    objects.append(Ball(Vector3(1, 10, 0), (255, 0, 0)))
    objects.append(Ball(Vector3(0, 20, 1), (0, 255, 0)))
    objects.append(Ball(Vector3(0, 30, 1), (255, 255, 0)))
    objects.append(Cube(Vector3(0, 40, 1), (0, 255, 255, 0)))

    for o in objects:
        dynamicsWorld.addRigidBody(o.body)

    simulate(dynamicsWorld, objects, debug)


class Ground:
    def __init__(self):
        self.boxHalfExtents = Vector3(20, 2, 20)
        groundShape = BoxShape(self.boxHalfExtents)
        groundTransform = Transform()
        groundTransform.setIdentity()
        groundTransform.setOrigin(Vector3(0, -4, 0))
        groundMotion = DefaultMotionState()
        groundMotion.setWorldTransform(groundTransform)
        self.body = RigidBody(groundMotion, groundShape)
        self.body.setRestitution(0.5)
        self.motion = groundMotion


    def render(self):
        x, y, z = (
            self.boxHalfExtents.x, self.boxHalfExtents.y, self.boxHalfExtents.z)
        o = self.motion.getWorldTransform().getOrigin()
        glColor(0, 0, 255)
        glTranslate(o.x, o.y, o.z)
        glBegin(GL_TRIANGLE_STRIP)
        glVertex(-x, y, -z)
        glVertex(x, y, -z)
        glVertex(-x, y, z)
        glVertex(x, y, z)
        glEnd()



class Ball:
    def __init__(self, position, color, radius=2):
        self.radius = radius
        ballShape = SphereShape(self.radius)
        ballTransform = Transform()
        ballTransform.setIdentity()
        ballTransform.setOrigin(position)
        ballMotion = DefaultMotionState()
        ballMotion.setWorldTransform(ballTransform)
        self.body = RigidBody(ballMotion, ballShape, 2.0)
        self.body.setRestitution(0.9)
        self.motion = ballMotion
        self.quad = gluNewQuadric()
        self.color = color


    def render(self):
        o = self.motion.getWorldTransform().getOrigin()
        glColor(*self.color)
        glTranslate(o.x, o.y, o.z)
        gluSphere(self.quad, self.radius, 25, 25)


class Cube:
    def __init__(self, position, color, length=5):
        self.length = length
        cubeShape = BoxShape(Vector3(length/2.0,length/2.0,length/2.0))
        cubeTransform = Transform()
        cubeTransform.setIdentity()
        cubeTransform.setOrigin(position)
        cubeMotion = DefaultMotionState()
        cubeMotion.setWorldTransform(cubeTransform)
        self.body = RigidBody(cubeMotion, cubeShape, 2.0)
        self.body.setRestitution(0.9)
        self.motion = cubeMotion
        self.quad = gluNewQuadric()
        self.color = color


    def render(self):
        o = self.motion.getWorldTransform().getOrigin()
        x = self.motion.getWorldTransform().getRotation()
        glColor(*self.color)
        glTranslate(o.x, o.y, o.z)
        glRotate(x.getAngle()*57.2957795,x.getAxis().x,x.getAxis().y,x.getAxis().z)
        glutSolidCube(5.0)



def step(world):
    timeStep = fixedTimeStep = 1.0 / 60.0
    world.stepSimulation(timeStep, 1, fixedTimeStep)
    now = time.time()
    delay = now % timeStep
    time.sleep(delay)


def render(objects):
    glPushMatrix()
    for o in objects:
        glPushMatrix()
        o.render()
        glPopMatrix()
    glPopMatrix()

    pygame.display.flip()
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


def simulate(world, objects, debug):
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
        
        render(objects)
        
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
        
if __name__ == '__main__':
    main()
