# -*- coding: utf-8 -*-
"""
Created on Sun Feb 21 17:28:16 2021
Jugando con las ideas de Lara para implementar algoritmos de busqueda basados
en el gradiante y el hessiano
@author: abierto
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def gausianillas(x,y,c,s,th,p):
    """
    Devuelve el valor de una gausiana definida en dos dimesiones x e y.
    Cada dimension tiene su propia desviación. La gausisana puede tener su
    centro en cualquier valor arbitrario y estar orientada 
    también de modo arbitrario
    x e y pueden ser:
    un par de puntos en los que calcular el valor V que toma la gausiana
    Un par de matrices generadas por meshgrid para crear una malla de puntos
    En este caso V es una matriz de la misma dimension que x e y con los
    valores que toma v en los  nodos de la malla
    c =[cx,cy]. representa el centro de la gausiana debe ser un array de 
    dos elementos
    s = [sx,sy] desviación en la dirección xy, en lo que podríamos llamar
    'ejes gaussiana'
    th angulo de los 'ejes gaussiana' respecto a los ejes tierra.
    p modulo de la gausiana (valor que toma en el centro)
    
    Return -> V (mirar definición en el código de la función línea 41)
    """

    x0 = x-c[0]
    y0 = y-c[1]
    cth = np.cos(th)
    sth = np.sin(th)
    R =np.array([x0,y0])
    Rot = np.array([[cth,sth],[-sth,cth]]) #matriz de rotación
    Q = np.array([[1/(2*s[0]**2),0],[0,1/(2*s[1]**2)]])
    if x0.ndim > 1:       
        Rrt = np.dot(Rot,R.transpose(1,0,2))        
    else:
        Rrt = np.dot(Rot,R)
   
    #V = p*np.exp(-(Rrt[0]**2/(2*s[0]**2)+Rrt[1]**2/(2*s[1]**2)))
    H = np.sum(Rrt*np.dot(Q,Rrt.transpose(1,0,2)),axis=0)
    V =p*np.exp(-H)
    return V


#fig = plt.figure()
#ax = plt.axes(projection='3d')

#x = np.arange(-1.,1.,0.01)
#y = np.arange(-1.,1,0.01)

#[X,Y] = np.meshgrid(x,y)
# Z1 = gausianillas(X,Y,[0,0],[0.1,0.2],0,1)
# Z2 = gausianillas(X,Y,[0.5,0.2],[0.1,0.2],np.pi/3,.5)
# Z3 = gausianillas(X,Y,[-0.9,-0.2],[0.2,0.4],np.pi/2,2)
# Z  = Z1 + Z2 + Z3
#ax.plot_surface(X,Y,Z,cmap= 'coolwarm')
#ax.set_xlabel('x')
#ax.set_ylabel('y')
#ax.set_zlabel('z')

