class Renderable(object):
    def __init__(self, obj, color):
        self.color = color
    
    def render(self):
        pass
        

class RenderableBox(Renderable):
    pass
    

def makeRenderable(obj, color):
    return Renderable(obj, color)
