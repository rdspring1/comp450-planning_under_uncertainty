#!/usr/bin/python

from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos

hside = 0.025
lineSegs = [[[0, 0], [hside, 0]], [[hside, 0], [hside, hside]], [[hside, hside], [0, hside]], [[0, hside], [0, 0]]]

def plot(path, obstacles):
    fig = plt.figure()
    ax = fig.gca()
    x0, y0, dx, dy = ax.get_position().bounds
    maxd = max(dx, dy)
    width = 6 * maxd / dx
    height = 6 * maxd / dy
    fig.set_size_inches((width, height))

    for i in range(len(obstacles)):
        ax.add_patch(patches.Polygon(obstacles[i], fill=True, color='0.20'))

    # Plotting the path (reference point)
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    # Plotting the square
    for lineSeg in lineSegs:
        linePath = []
        for p in path:
            x = []
            y = []
            for v in lineSeg:
                x.append(v[0] * cos(p[2]) - v[1] * sin(p[2]) + p[0])
                y.append(v[0] * sin(p[2]) + v[1] * cos(p[2]) + p[1])
            ax.plot(x, y, 'k')

    plt.axis([0,1,0,1])
    plt.show()

def readEnv(env):
    filename = "./environments/env" + env + ".txt"
    lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]
    obstacles = []
    polygon = []
    start = False
    for n in range(len(lines)):
        if lines[n] == "end":
            start = True
        elif lines[n] == "ep":
            obstacles.append(polygon)
            polygon = []
        elif start:
            polygon.append(tuple([float(x) for x in lines[n].split(' ')]))
    return obstacles 

# Read the cspace definition and the path from filename
def readPath(filename):
    lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]

    if len(lines) == 0:
        print "Empty File"
        sys.exit(1)

    env = lines[0].strip()
    obstacles = readEnv(env)
    data = [[float(x) for x in line.split(' ')] for line in lines[1:]]
    return obstacles, data

if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'path.txt'

    obstacles, path = readPath(filename)
    try:
        plot(path, obstacles)
    except IndexError:
        print "Unknown Environment" + env
        sys.exit(1)
