#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 22 16:04:38 2021
fidling around with the gradient estimation
@author: juanjimenez
"""
import numpy as np
import matplotlib.pyplot as pl
from gauss_dist import gausianillas

#Uniformed Distributed circular formation for N agents########################
N = 10 #Numero de robotillos

e =np.array([[1],[0]]) # x-axis  SO defined by Lara to perform her computations

ctr = np.array([[0],[0]]) #formation center

#agents' angular distribution (uniformed distributed around a circle)  
phi_N = np.zeros([N])
for i in range(N):
    phi_N[i] =(i+1)*2*np.pi/N

D = 0.5 #formation radious
w = 1 #angular velocity (common for all agents)
##############################################################################

#Gaussian signal distribution#################################################
cd  = np.array([[0],[0]]) #signal center
sig = [0.5,0.3] #gaussian deviation in x, y
th = np.pi/3
p = 1
pl.figure(1)

t = np.arange(0,10.01,0.01)
for i in t:
    phi_t = phi_N + w*i #agents' angles 
    c = np.cos(phi_t)
    s = np.sin(phi_t)
    R = np.array([[c,-s],[s,c]]) #Rotation matrix
    r = ctr + D*np.dot(R.T,e) # agents' positions 
    ''''
    I don's 'stack' the vectors. Instead, I follow a 'Pseudo tensorial'
    arranging. So, I end up wiht a matrix of rotation matrices (2x2xN).
    To rotate e =[1,0]T, fist we need to transpose R^T (NX2X2) An then
    R^T_ijk*e_js= r_iks r(N,2,1) its an array of N position vectors (x,y) 
    '''
    if i%2==0:
     #estimating the gradient at times 0,2,4,6....   
     pl.plot(r[:,0],r[:,1])
     sigmai = gausianillas(r[:,0],r[:,1],cd,sig,th,p)
     grdst = np.sum(sigmai.T*(r-ctr).T,2)*2/N/D**2
     print(grdst)
     pl.arrow(ctr[0,0],ctr[1,0],grdst[0,0],grdst[0,1])
    

x = np.arange(-2.,2.,0.01)
y = np.arange(-2.,2,0.01)
[X,Y] = np.meshgrid(x,y)
Z = gausianillas(X,Y,cd,sig,th,p)
pl.contour(X,Y,Z,np.arange(0.001,1,0.1))
pl.axis('equal')


fig = pl.figure(2)
ax = pl.axes(projection='3d')




ax.plot_surface(X,Y,Z,cmap= 'coolwarm')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')   