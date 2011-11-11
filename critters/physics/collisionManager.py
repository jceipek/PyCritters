'''
Created on Nov 10, 2011

@author: wdolphin
'''

class CollisionManager(object):
    '''
    This object manages collision groups for RigidBodies by providing a simple
    interface to declare collisions between pairs of rigid bodies as ignored by
    the physics engine. Currently this manager is limited to a maximum of 16
    Rigid Bodies, this limitation may be lifted with the implementation of
    BroadPhaseFilter.
    '''

    MAX_SIZE = 16 #limitation enforced by the number of bits in a short, may change later.
    
    def __init__(self,physicsObjectList=None):
        '''
        Initializes an empty CollisionManager, optionally if a list of PhysicsObjects are supplied,
        the dictionary is initialized to contain those physicsObjects with NO IGNORED collisions.
        '''
        self.ignores = dict()
        self.bits = dict()
        
        if physicsObjectList != None:
            if len(physicsObjectList) > CollisionManager.MAX_SIZE:
                raise ValueError('The maximum number of Rigid Bodies which may be mapped is 16', len(physicsObjectList), '>16')        

            for physicsObject in physicsObjectList:
                t_id = physicsObject.identifier
                if t_id in self.ignores:
                    raise ValueError('The list of PhysicsObjects may not contain clones, alternatively, identifiers were not unique.')        
                self.ignores[t_id] = [] #each PhysicsObject ignores no collisions by default
                
                
    def addPhysicsObject(self,physObj):
        '''
        Adds a PhysicsObject to the collisionManager but does not imply any
        relations. This is especially useful for things like the ground.
        '''
        if len(self.ignores) < CollisionManager.MAX_SIZE:
            self.ignores[physObj.identifier] = []
        else:
            raise ValueError('The maximum number of Rigid Bodies which may be mapped is 16')        
        
    def ignoreCollision(self,physObj1,physObj2):
        '''
        Registers physObj1 and physObj2 to have their collisions be ignored by the physics engine. 
        This operation is atomic; if it fails it will not have any side effects
        '''
        _added1 = False
        id1 = physObj1.identifier
        id2 = physObj2.identifier
        if not id1 in self.ignores:
            if len(self.ignores) < CollisionManager.MAX_SIZE:
                self.ignores[id1] = []
                _added1 = True
            else:
                raise ValueError('The maximum number of Rigid Bodies which may be mapped is 16')        
        if not id2 in self.ignores:
            if len(self.ignores) < CollisionManager.MAX_SIZE:
                self.ignores[id2] = []
            else:
                if _added1:
                    del self.ignores[id1] #enforce atomic operation
                raise ValueError('The maximum number of Rigid Bodies which may be mapped is 16')            
        self.ignores[id1].append(id2)
        self.ignores[id2].append(id1)
    
    def calculateCollisionGroups(self):
        '''
        Calculates the bits for the collision groups, should be run once all of the items have been added
        '''
        last = 0b01
        for item in self.ignores.iterkeys():
            self.bits[item] = last
            last =last <<1 #move on to the next bit
            #TODO test for out of range
    
    def getCollisionFilterGroup(self,physObj):
        '''
        @return: the calculated collisionFilterGroup for the physObj, an integer between 01 and 2^16 -1
        '''
        if not physObj.identifier in self.bits:
            raise ValueError('The Collision Groups must be calculated before this function is invoked')            
        return self.bits[physObj.identifier]

    def getCollisionFilterMask(self,physObj):
        '''
        @return: the calculated collisionFilterMask, the result of binary or'ing all PhysicsObjects this physObj has been told to ignore
        '''
        if not physObj.identifier in self.bits:
            raise ValueError('The Collision Groups must be calculated before this function is invoked')            
        else:
            #itemsToColideWith is the list of all elements which must be or'ed to create the mask
            toColideWith = [self.bits[item] for item in self.bits.iterkeys() if item not in self.ignores[physObj.identifier]]
            return reduce(lambda x,y: x|y,toColideWith)

if __name__ =='__main__':
    import random
    class obj:
        def __init__(self):
            self.identifier = random.randint(0,20)
            
    cm = CollisionManager([obj(),obj()])