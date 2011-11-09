'''
Created on Nov 8, 2011

@author: wdolphin
'''

class BodyPart(object):
    '''
    An abstraction of a virtual creature's body part. 
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
    
    