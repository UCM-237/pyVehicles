# -*- coding: utf-8 -*-
"""
Created on Tue Mar 23 19:48:44 2021
Prueba a ver si los ordenamos por el angulo de inicio
@author: abierto
"""
import sys
# Add paths
sys.path.insert(1, './GNC')
sys.path.insert(1, './graphics')
sys.path.insert(1, './postproc')
sys.path.insert(1, './vehicles')

import pygame
import matplotlib.pyplot as pl
import drawmisc
import agents as ag
import numpy as np
import logpostpro as lp
import gvf

# setup simulation
WIDTH = 1000
HEIGHT = 1000

CENTERX = WIDTH/2
CENTERY = WIDTH/2

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

size = [WIDTH, HEIGHT]
#screen = pygame.display.set_mode(size)

num_of_agents = 3
list_of_agents = []

for i in range(num_of_agents):
    if i == 1:
        color = RED
    elif i == 2:
        color = GREEN
    elif i == 3:
        color = BLUE
    else:
        color = WHITE

    list_of_agents.append(ag.AgentUnicycle(color, i, 1000*np.random.rand(2,1), 50-100*np.random.rand(2,1)))
phi=[]
for i in list_of_agents:
    phi.append(np.arctan2(i.pos[1,0],i.pos[0,0]))
  
for i in list_of_agent:   
    mini = phi.index(min(phi))
    newinde