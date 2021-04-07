import sys
# Add paths
sys.path.insert(1, './GNC')
sys.path.insert(1, './graphics')
sys.path.insert(1, './postproc')
sys.path.insert(1, './vehicles')

import pygame
import matplotlib.pyplot as pl
import matplotlib.colors as mcolors
import drawmisc
import agents as ag
import numpy as np
from numpy import linalg as la

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

# Network
num_of_agents = 16
list_of_agents = []
list_of_edges = [] # For the coordination on the circle

for i in range(num_of_agents-1):
  list_of_edges.append((i,i+1))    # Daisy chain network topology

desired_inter_vehicle_theta = (2*np.pi/num_of_agents)*np.ones((len(list_of_edges),1))
theta_vehicles = np.zeros((num_of_agents,1))
k_coord = 100

# Incidence matrix
B = np.zeros((num_of_agents, len(list_of_edges)))
for idx,edge in enumerate(list_of_edges):
    B[edge[0],idx] =  1
    B[edge[1],idx] = -1

B_dir = np.zeros((num_of_agents, len(list_of_edges)))
for idx,edge in enumerate(list_of_edges):
    B_dir[edge[0],idx] =  1
    B_dir[edge[1],idx] =  0

# Directed vs Undirected graph. They have different convergence properties
# If you want to control with an undirected graph, then uncomment the following.
B_dir = B

for i in range(num_of_agents):
    theta_o = np.pi - (2*np.pi)*np.random.rand(1);
    list_of_agents.append(ag.AgentUnicycle(list(pygame.colordict.THECOLORS.items())[np.random.randint(0,657)][1], i, 1000*np.random.rand(2,1), 60*np.array([[np.cos(theta_o[0])],[np.sin(theta_o[0])]])))

for agent in list_of_agents:
    agent.traj_draw = True

# GVF
ke_circle = 5e-5
kd_circle = 60

xo = 500 # Circle's center
yo = 500
ro = 150 # radius

#
direction = -1 # Clock or counter-clock wise. This defines what angular velocity is positive

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

    # Coordination algorithm on the circle (calculated in a compact way)
    for idx,agent in enumerate(list_of_agents):
        theta_vehicles[idx] = np.arctan2(agent.pos[1]-yo, agent.pos[0]-xo)

    inter_theta = B.transpose().dot(theta_vehicles)
    error_theta = inter_theta - desired_inter_vehicle_theta

    if np.size(error_theta) > 1:
      for i in range(0, np.size(error_theta)):
        if error_theta[i] > np.pi:
          error_theta[i] = error_theta[i] - 2*np.pi
        elif error_theta[i] <= -np.pi:
          error_theta[i] = error_theta[i] + 2*np.pi
    else:
        if error_theta > np.pi:
          error_theta = error_theta - 2*np.pi
        elif error_theta <= -np.pi:
          error_theta = error_theta + 2*np.pi

    dr = -k_coord*B_dir.dot(np.sin(error_theta))
    dr = direction*dr

    # Guiding vector field
    for idx,agent in enumerate(list_of_agents):
        agent.draw(screen)
        circle_path = gvf.Path_gvf_circle(xo, yo, ro+dr[idx])
        ut = gvf.gvf_control_2D_unicycle(agent.pos, agent.vel, ke_circle, kd_circle, circle_path, direction)
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
    ax.grid
#circle = pl.Circle((xo, yo), ro, color='r')
#ax.add_patch(circle)

pl.show()
