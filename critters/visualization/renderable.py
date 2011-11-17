from OpenGL.GL import (
    GL_LINES,
    glColor,
    glBegin, glEnd, glVertex)
from bullet.bullet import Hinge2Constraint

class Renderable(object):
    def __init__(self, obj, color):
        self.color = color
    
    def render(self):
        pass
        

class RenderableBox(Renderable):
    pass
    
class RenderableConnection(Renderable):
    def __init__(self, obj, color):
        self.po1, self.po2, self.local1, self.local2 = obj
        if color == None:
            color = (1.0, 0.0, 0.0)
        Renderable.__init__(self, None, color)

    def render(self):
        glBegin(GL_LINES)
        v1 = self.po1.body.getWorldTransform().getOrigin() - self.local1
        v2 = self.po2.body.getWorldTransform().getOrigin() - self.local2
        glVertex(*v1)
        glVertex(*v2)
        glEnd()

def makeRenderable(obj, color=None):
    if(isinstance(obj, tuple)):
        return RenderableConnection(obj, color)
    return Renderable(obj, color)
