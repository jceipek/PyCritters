
import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

from physics.simulationEnvironment import  SimulationEnvironment
from physics.objects import Box
from bullet.bullet import Vector3


simEnv= SimulationEnvironment()


box1 = Box(Vector3(0, 2.5, 0), Vector3(9.0,5.0,5.0))

box2 = Box(Vector3(9.0, 2.5, 0), Vector3(9.0,5.0,5.0))


simEnv.addPhysicsObject(box1)
simEnv.addPhysicsObject(box2)

simEnv.addHinge(box1, box2, Vector3(4.5,0,0),Vector3(0,0,1),Vector3(0,1,0))

motors = [simEnv.getConstraint(box1,box2).getRotationalLimitMotor(2)]
hinge =simEnv.getConstraint(box1,box2)
hinge.getRotationalLimitMotor(0).hiLimit = 0
hinge.getRotationalLimitMotor(0).loLimit = 0
hinge.getRotationalLimitMotor(1).hiLimit = 0
hinge.getRotationalLimitMotor(1).loLimit = 0

for motor in motors:
    motor.enableMotor = True
    motor.targetVelocity = 10
    motor.hiLimit = 90
    motor.loLimit = -90
    motor.maxMotorForce = 50
    print(motor.maxMotorForce)
    #print(motor.currentPosition)



simEnv.run()
                
