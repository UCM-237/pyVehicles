import numpy as np
import agents as ag
from matplotlib.path import Path
import matplotlib.patches as patches

WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

colors = {WHITE: 'k', BLUE: 'b', GREEN: 'g', RED: 'r'}

def unicycle_patch(XY, yaw, color):
    Rot = np.array([[np.cos(yaw), np.sin(yaw)],[-np.sin(yaw), np.cos(yaw)]])

    apex = 45*np.pi/180 # 30 degrees apex angle
    b = np.sqrt(1) / np.sin(apex)
    a = b*np.sin(apex/2)
    h = b*np.cos(apex/2)

    z1 = 20*np.array([a/2, -h*0.3])
    z2 = 20*np.array([-a/2, -h*0.3])
    z3 = 20*np.array([0, h*0.6])

    z1 = Rot.dot(z1)
    z2 = Rot.dot(z2)
    z3 = Rot.dot(z3)

    verts = [(XY[0]+z1[1], XY[1]+z1[0]), \
             (XY[0]+z2[1], XY[1]+z2[0]), \
             (XY[0]+z3[1], XY[1]+z3[0]), \
             (0, 0)]

    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
    path = Path(verts, codes)

    return patches.PathPatch(path, facecolor=color, lw=2)

def plot_position(axis, agent):
    color = colors[agent.color]

    if(isinstance(agent, ag.AgentUnicycle)):
        axis.plot(np.trim_zeros(agent.log_pos[:,0], 'b'), np.trim_zeros(agent.log_pos[:,1], 'b'), color)
        up = unicycle_patch(agent.log_pos[agent.log_index-1, :], agent.log_theta[agent.log_index-1], color)
        axis.add_patch(up)
    else:
        axis.plot(np.trim_zeros(agent.log_pos[:,0], 'b'), np.trim_zeros(agent.log_pos[:,1], 'b'), color)
        axis.plot(agent.log_pos[0, 0], agent.log_pos[0, 1], 'x'+color)
        axis.plot(agent.log_pos[agent.log_index-1, 0], agent.log_pos[agent.log_index-1, 1], 'o'+color)

def plot_trajectories(axis, listofagents, B):
    p_final = []
    for agent in listofagents:
        color = colors[agent.color]
        axis.plot(np.trim_zeros(agent.log_pos[:,0], 'b'), np.trim_zeros(agent.log_pos[:,1], 'b'), color)
        axis.plot(agent.log_pos[0, 0], agent.log_pos[0, 1], 'x'+color)
        axis.plot(agent.log_pos[agent.log_index-1, 0], agent.log_pos[agent.log_index-1, 1], 'o'+color)
        p_final.append((agent.log_pos[agent.log_index-1, 0], agent.log_pos[agent.log_index-1, 1]))

    X = np.asarray(p_final)
    X = X.reshape(2*len(listofagents),1)

    agents, edges = B.shape
    a, b = 0, 0
    for i in range(0, edges):
        for j in range(0, agents):
            if B[j,i] == 1:
                a = j
            elif B[j,i] == -1:
                b = j
        axis.plot([X[2*a], X[2*b]], [X[2*a+1], X[2*b+1]], 'k--', lw=1.5)



