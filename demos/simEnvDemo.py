"""
"""

from critters.physics.simulationEnvironment import SimulationEnvironment
from critters.physics import objects 

simEnv = SimulationEnvironment(vis=True, gravity=True)

cube = objects.Rect((0,0), (1,1))

simEnv.addPhysicsObject(cube)

simEnv.run()
