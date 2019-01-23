import triangle
import triangle.plot
import numpy as np
import matplotlib.pyplot as plt
import math
import pickle
import argparse
import sys
from decimal import Decimal

#2D ellipsoid triangles
pi = 3.1415926

class Ellipsoid_2D:
    def __init__(self, semi_axes_, theta_, epsilon_, num_, center_):
        self.semi_axes = semi_axes_
        self.theta = theta_
        self.epsilon = epsilon_
        self.num = num_
        self.center = center_
    def getPointsBoundary(self):
        trans = []
        X = []
        for i in range(self.num):
            th = (2 * pi * i) / (self.num-1)
            x = [self.semi_axes[0] * self.expFun(th,self.epsilon,0), self.semi_axes[1] * self.expFun(th, self.epsilon,1)]
            trans.append([self.center[0], self.center[1]])
            X.append([x[0], x[1]])
        X = self.transformation_2D(self.theta, X, trans)
        return X
    def transformation_2D(self, theta_, X_, trans_):
        rotations = []
        rotation_mat = self.getRotation_Matrix(theta_)
        for i in range(len(X_)):
            aux = np.matmul(rotation_mat, np.asmatrix(X_[i]).transpose())
            aux = aux + np.asmatrix(trans_[i]).transpose()
            rotations.append([aux[0,0], aux[1,0]])
        return np.asmatrix(rotations)
    def getRotation_Matrix(self, theta_):
        S = np.matrix([[0, -1], [1, 0]])
        R = np.identity(2) + math.sin(theta_) * S + (1 - math.cos(theta_)) * (S * S)
        return R
    def expFun(self, theta_, p_, func_):
        if func_ == 0:
            return np.sign(math.cos(theta_)) * math.pow(Decimal(abs(math.cos(theta_))),Decimal(p_))
        else:
            return np.sign(math.sin(theta_)) * math.pow(Decimal(abs(math.sin(theta_))),Decimal(p_))


############### Main

def main(axis_x, axis_y, epsilon, num, file):
    ellip2D = Ellipsoid_2D([axis_x, axis_y], 0, epsilon, num, [0.0, 0.0])
    points = ellip2D.getPointsBoundary()
    x = points[:,0]
    y = points[:,1]
    plt.scatter(np.asarray(x),np.asarray(y))
    plt.show()
    val = list(zip(x,y))
    A = dict(vertices=np.array(val))
    B = triangle.triangulate(A)
    vert = B['vertices']
    vertices_file = open("vertices_" + file+".txt", "w")
    for i in range(len(vert)):
        stringVertex = ' '.join(map(str, vert[i,:]))
        vertices_file.write(stringVertex + "\n")
    vertices_file.close()
    tri = B['triangles']
    triangle_file = open("triangle_"+ file +".txt", "w")
    for i in range(len(tri)):
        stringVertex = ' '.join(map(str, tri[i, :]))
        triangle_file.write(stringVertex + "\n")
    triangle_file.close()


configurations = [[4,2,1,50], [0.8,0.5,1,50]]

for i in range(len(configurations)):
    main(configurations[i][0],configurations[i][1],configurations[i][2],configurations[i][3],"obs"+str(i))

main(5,3,1,50,"agent")
main(40,30,0.001,50,"arena")

