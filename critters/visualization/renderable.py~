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
    def __init__(self, hinge, color):
        Renderable.__init__(self, hinge, color)
        self.anchor = hinge.getAnchor()
        self.anchor2 = hinge.getAnchor2()


    def render(self):
        glBegin(GL_LINES)
            glColor(self.color)
            glVertex(self.anchor)
            glVertex(self.anchor2)
        glEnd()

def makeRenderable(obj, color):
    if(isInstance(obj, Hinge2Constraint)):
        return RenderableConnection(obj, color)
    return Renderable(obj, color)
