#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
"""

import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

from physics.simulationEnvironment import SimulationEnvironment
from physics import objects 


simEnv = SimulationEnvironment(vis=True, gravity=True)
ground= objects.StaticRect(position=(0,1),size=(50,1))

cube1 = objects.Rect((2.0,2.0), (1,1))
cube2 = objects.Rect((4.0,2.0), (1,1))

simEnv.addPhysicsObject(cube1)
simEnv.addPhysicsObject(cube2)
simEnv.addPhysicsObject(ground)

joint = simEnv._addHinge(cube1,cube2,(3,3))
joint.motorSpeed = 2
simEnv.run()
