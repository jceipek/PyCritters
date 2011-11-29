#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
"""
import pygame
from Box2D import b2# This maps Box2D.b2Vec2 to vec2 (and so on)
import math

# --- constants ---
# Box2D deals with meters, but we want to display pixels, 
# so define a conversion factor:
PPM=20.0 # pixels per meter
TARGET_FPS=60
TIME_STEP=1.0/TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT=640,480

# --- pygame setup ---
screen=pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock=pygame.time.Clock()


# --- pybox2d world setup ---
# Create the world
world=b2.world(gravity=(0,-0),doSleep=True)

# And a static body to hold the ground shape
ground_body=world.CreateStaticBody(
    position=(0,1),
    shapes=b2.polygonShape(box=(50,1)),
    )


# Create a dynamic body
box1=world.CreateDynamicBody(position=(10,3+5), angle=0.0)
box2=world.CreateDynamicBody(position=(14,5+5), angle=0.0)

rj=world.CreateRevoluteJoint(
    bodyA=box1, 
    bodyB=box2, 
    anchor=(12,4+5),
    lowerAngle = -.5 * b2.pi, # -90 degrees
    upperAngle = .5 * b2.pi, #  45 degrees
    enableLimit = True,
    maxMotorTorque = 200.0,
    motorSpeed = 1,
    enableMotor = True,
    )

# And add a box fixture onto it (with a nonzero density, so it will move)
box1.CreatePolygonFixture(box=(2,1), density=1000, friction=0.3)
box2.CreatePolygonFixture(box=(2,1), density=1, friction=0.3)
colors = {
    b2.staticBody  : (255,255,255,255),
    b2.dynamicBody : (127,127,127,255),
    }

# --- main game loop ---
running=True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if event.type==pygame.QUIT or (event.type==pygame.KEYDOWN and event.key==pygame.K_ESCAPE):
            # The user closed the window or pressed escape
            running=False

    screen.fill((0,0,0,0))
    # Draw the world
    for body in world.bodies: # or: world.bodies
        # The body gives us the position and angle of its shapes
        for fixture in body.fixtures:
            # The fixture holds information like density and friction,
            # and also the shape.
            shape=fixture.shape
            
            # Naively assume that this is a polygon shape. (not good normally!)
            # We take the body's transform and multiply it with each 
            # vertex, and then convert from meters to pixels with the scale
            # factor. 
            vertices=[(body.transform*v)*PPM for v in shape.vertices]

            # But wait! It's upside-down! Pygame and Box2D orient their
            # axes in different ways. Box2D is just like how you learned
            # in high school, with positive x and y directions going
            # right and up. Pygame, on the other hand, increases in the
            # right and downward directions. This means we must flip
            # the y components.
            vertices=[(v[0], SCREEN_HEIGHT-v[1]) for v in vertices]

            pygame.draw.polygon(screen, colors[body.type], vertices)

    # Make Box2D simulate the physics of our world for one step.
    # Instruct the world to perform a single step of simulation. It is
    # generally best to keep the time step and iterations fixed.
    # See the manual (Section "Simulating the World") for further discussion
    # on these parameters and their implications.
    world.Step(TIME_STEP, 10, 10)

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)
    
pygame.quit()
print('Done!')
