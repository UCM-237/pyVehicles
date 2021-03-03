import numpy as np
from numpy import linalg as la

class Path_gvf:
    def __init__(self):
        self.e = 0
        self.grad = np.zeros((2,1))
        self.Hessian = np.zeros((2,2))

class Path_gvf_circle(Path_gvf):
    def __init__(self, xo, yo, r):
        Path_gvf.__init__(self)
        self.xo = xo
        self.yo = yo
        self.r = r

    def calculate_e_grad_Hess(self, p):
        xel = (p[0][0] - self.xo)
        yel = (p[1][0] - self.yo)
        self.e = xel**2 + yel**2 - self.r**2
        self.grad = np.array([[2*xel],[2*yel]])
        self.Hessian = 2*np.eye(2);

def gvf_control_2D(p, dot_p, ke, kd, Path, direction):
    Path.calculate_e_grad_Hess(p)
    e = Path.e
    n = Path.grad
    H = Path.Hessian
    E = np.array([[0, 1],[-1, 0]])
    tau = direction*E.dot(n)

    dot_pd = tau - ke*e*n
    ddot_pd = (E - ke*e*np.eye(2)).dot(H).dot(dot_p) - ke*n.T.dot(dot_p) * n
    ddot_pdhat = -E.dot(dot_pd.dot(dot_pd.T)).dot(E).dot(ddot_pd) / np.linalg.norm(dot_pd)**3

    dot_Xid = ddot_pdhat.T.dot(E).dot(dot_pd) / np.linalg.norm(dot_pd)

    u_theta = dot_Xid + kd*dot_p.T.dot(E).dot(dot_pd)/(np.linalg.norm(dot_p)*np.linalg.norm(dot_pd))

    return float(u_theta)
