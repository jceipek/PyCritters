#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
"""

import sys
sys.path.append('../critters')  #add critters to pythonpath to use as library

from physics.simulationEnvironment import SimulationEnvironment
from physics import objects 

# --- constants ---
# Box2D deals with meters, but we want to display pixels, 
# so define a conversion factor:
TARGET_FPS=60
TIME_STEP=1.0/TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT=640,480

simEnv = SimulationEnvironment(vis=True, gravity=True)

cube = objects.Rect((0,0), (1,1))

simEnv.addPhysicsObject(cube)

simEnv.run()
