#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
"""

import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

from physics.simulationEnvironment import SimulationEnvironment
from physics import objects 


simEnv = SimulationEnvironment(vis=True, gravity=True)

cube = objects.Rect((5,5), (1,1))

simEnv.addPhysicsObject(cube)

simEnv.run()
