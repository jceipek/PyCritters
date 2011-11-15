
import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

from physics.simulationEnvironment import  SimulationEnvironment
from physics.objects import Box
from bullet.bullet import Vector3


simEnv= SimulationEnvironment()

box1 = Box(Vector3(0, 2.5, 0), Vector3(9.0,5.0,5.0))
box2 = Box(Vector3(5, 19.5, 0), Vector3(9.0,5.0,5.0))
box3 = Box(Vector3(2.5, 35.5, 0), Vector3(9.0,5.0,5.0))


simEnv.addPhysicsObject(box1)
simEnv.addPhysicsObject(box2)
simEnv.addPhysicsObject(box3)

simEnv.ignoreCollision(box3,box2)
simEnv.ignoreCollision(box1,box2)
for obj in simEnv.objectList:
    print(obj)

simEnv.run()
                
                
