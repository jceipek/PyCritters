#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
"""

import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

from physics.simulationEnvironment import SimulationEnvironment
from physics import objects 


simEnv = SimulationEnvironment(vis=True, gravity=False)

cube1 = objects.Rect((1.0,10.5), (1,1))

simEnv.addPhysicsObject(cube1)


simEnv.run()
