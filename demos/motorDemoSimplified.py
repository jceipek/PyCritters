"""
"""
from critters.physics.simulationEnvironment import SimulationEnvironment
from critters.physics import objects 


simEnv = SimulationEnvironment(vis=True, gravity=True)
ground= objects.StaticRect(position=(0,1),size=(50,1))

cube1 = objects.Rect((2.0,2.0), size=(1,1))
cube2 = objects.Rect((3.0,2.0), size=(1,1))

simEnv.addPhysicsObject(cube1)
simEnv.addPhysicsObject(cube2)
simEnv.addPhysicsObject(ground)

joint = simEnv._addHinge(cube1,cube2,(2.5,2.5))
joint.motorSpeed = 2
simEnv.run()
