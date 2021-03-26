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

size = [WIDTH, HEIGHT]
screen = pygame.display.set_mode(size)

num_of_agents = 3
list_of_agents = []

for i in range(num_of_agents):
    theta_o = np.pi - (2*np.pi)*np.random.rand(1);
    list_of_agents.append(ag.AgentUnicycle(list(pygame.colordict.THECOLORS.items())[np.random.randint(0,657)][1], i, 1000*np.random.rand(2,1), 60*np.array([[np.cos(theta_o[0])],[np.sin(theta_o[0])]])))

for agent in list_of_agents:
    agent.traj_draw = True

# GVF
ke_circle = 5e-5
kd_circle = 60
circle_path = gvf.Path_gvf_circle(500,500,100)

# run simulation
pygame.init()
clock = pygame.time.Clock()
fps = 50
dt = 1.0/fps
time = 0

runsim = True
while(runsim):
    screen.fill(BLACK)

    us = 0 # We keep constant velocity

    for agent in list_of_agents:
        agent.draw(screen)
        ut = gvf.gvf_control_2D_unicycle(agent.pos, agent.vel, ke_circle, kd_circle, circle_path, 1)
        agent.step_dt(us, ut, dt)

    clock.tick(fps)
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            endtime = pygame.time.get_ticks()
            pygame.quit()
            runsim = False


# Postprocessing
fig = pl.figure(0)
ax = fig.add_subplot(111)
for agent in list_of_agents:
    lp.plot_position(ax, agent)
    ax.axis("equal")
pl.show()
