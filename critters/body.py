'''
Created on Nov 8, 2011

@author: wdolphin
'''

class BodyPart(object):
    '''
    An abstraction of a virtual creature's body part for the morphology tree. These are interpreted and converted to physical objects when simulated by TheHolyGrail 
    '''

    def __init__(self):
        '''
        Constructor
        '''
        self.physicsPart = None
        self.neuralNetwork = None
    
    def act(self):
        pass
    
    def think(self):
        pass
    

class Connection(object):
    '''
    '''
    def __init__(self,connectionValue1,connectionValue2,connectionType='hinge'):
        self.connectionValue1 = connectionValue1
        self.connectionValue2 = connectionValue2
        
    
