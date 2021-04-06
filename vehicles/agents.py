import pygame
import numpy as np
from numpy import linalg as la

def normalizeAngle(angle):
    newAngle = angle;
    while newAngle <= -np.pi:
        newAngle = newAngle + 2*np.pi;
    while newAngle > np.pi:
        newAngle = newAngle - 2*np.pi;
    return newAngle;

class Agent:
    def __init__(self, color, label, pos=np.zeros((2,1)), vel=np.zeros((2,1)), traj_capacity=250, log_capacity=30000):
        # States
        self.label = label # int
        self.pos = pos # ndarray float
        self.pos_old = pos # ndarray float
        self.vel = vel # ndarray float
        self.theta = np.arctan2(vel[1][0], vel[0][0])
        self.speed = np.linalg.norm(vel)

        self.neighbors = [] # list of neighbors

        self.color = color

        self.traj_draw = True
        self.traj_capacity = traj_capacity
        self.traj = np.zeros((self.traj_capacity, 2))
        self.traj_index = 0
        self.traj_full = False
        self.wrt_label = True

        self.log_capacity = log_capacity
        self.log_pos = np.zeros((self.log_capacity, 2))
        self.log_vel = np.zeros((self.log_capacity, 2))
        self.log_u = np.zeros((self.log_capacity, 2))
        self.log_theta = np.zeros(self.log_capacity)
        self.log_index = 0

        # Consensus
        self.consensus_kc = 1e-1
        self.consensus_kv = 1

        # Distance-based formation control
        self.distance_based_kc = 1e-1
        self.distance_based_kv = 1
        self.desired_distances = []


    def log_trajectory(self):
        self.traj[self.traj_index,:] = self.pos.T
        self.traj_index += 1
        if(self.traj_index >= self.traj_capacity):
            self.traj_full = True
            self.traj_index = 0

    def draw_trajectory(self, surf):
        if(self.traj_full):
            for point in self.traj:
                surf.set_at((int(point[0]), int(surf.get_height()-point[1])), self.color)
        else:
            for point in self.traj[:self.traj_index]:
                surf.set_at((int(point[0]), int(surf.get_height()-point[1])), self.color)

    def write_label(self,surf):
        font = pygame.font.Font('freesansbold.ttf',15)
        text = font.render(str(self.label), True, (255,255,255))
        textRect = text.get_rect()
        textRect.center = (int(self.pos[0]), int(surf.get_height()-self.pos[1]))
        surf.blit(text,textRect)

class AgentSI(Agent):
    def step_dt(self, u, dt):
        self.pos += u*dt

        self.log_trajectory()

        self.log_pos[self.log_index,:] = self.pos.T
        self.log_u[self.log_index,:] = u.T
        self.log_index += 1
        if(self.log_index >= self.log_capacity):
            self.log_index = 0

    def draw(self, surf):
        pygame.draw.circle(surf, self.color, (int(self.pos[0][0]),int(surf.get_height()-self.pos[1][0])), 4, 0)
        if(self.traj_draw):
            self.draw_trajectory(surf)


class AgentDI(Agent):
    def step_dt(self, u, dt):
        self.vel += u*dt
        self.pos += self.vel*dt

        self.log_trajectory()

        self.log_pos[self.log_index,:] = self.pos.T
        self.log_vel[self.log_index,:] = self.vel.T
        self.log_u[self.log_index,:] = u.T
        self.log_index += 1
        if(self.log_index >= self.log_capacity):
            self.log_index = 0

    def draw(self, surf):
        pygame.draw.circle(surf, self.color, (int(self.pos[0][0]),int(surf.get_height()-self.pos[1][0])), 4, 0)
        if(self.traj_draw):
            self.draw_trajectory(surf)

class AgentUnicycle(Agent):
    def __init__(self, color, label, pos=np.zeros(2), vel=np.zeros(2), traj_capacity=250, log_capacity=30000):
        Agent.__init__(self, color, label, pos, vel, traj_capacity, log_capacity)
        self.flock_kva = 1
        self.flock_ks = 0.3
        self.flock_kc = 1
        self.flock_ke = 1.3

    def step_dt(self, us, uw, dt):
        self.theta += uw*dt
        self.theta = normalizeAngle(self.theta)
        self.speed += us*dt
        self.vel = self.speed*np.array([[np.cos(self.theta)], [np.sin(self.theta)]])
        self.pos += self.vel*dt

        self.log_trajectory()

        self.log_pos[self.log_index,:] = self.pos.T
        self.log_vel[self.log_index,:] = self.vel.T
        self.log_u[self.log_index,:] = np.array([us,uw])
        self.log_theta[self.log_index] = self.theta
        self.log_index += 1
        if(self.log_index >= self.log_capacity):
            self.log_index = 0

    def draw(self, surf):
        yaw = -self.theta
        Rot = np.array([[np.cos(yaw), np.sin(yaw)],[-np.sin(yaw), np.cos(yaw)]])

        apex = 80*np.pi/180
        b = np.sqrt(2) / np.sin(apex)
        a = b*np.sin(apex/2)
        h = b*np.cos(apex/2)

        z1 = np.array([[-a/2], [h*0.3]])
        z2 = np.array([[-a/2], [-h*0.3]])
        z3 = np.array([[h*0.6], [0]])

        z1 = self.pos + 20*Rot.dot(z1)
        z2 = self.pos + 20*Rot.dot(z2)
        z3 = self.pos + 20*Rot.dot(z3)

        yoff = surf.get_height()

        tuple_of_corners = ((z1[0][0],yoff-z1[1][0]),(z2[0][0],yoff-z2[1][0]),(z3[0][0],yoff-z3[1][0]))
        pygame.draw.polygon(surf, self.color, tuple_of_corners)


        if(self.traj_draw):
            self.draw_trajectory(surf)
        if(self.wrt_label):
            self.write_label(surf)
