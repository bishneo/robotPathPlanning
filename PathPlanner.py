import AllConstants
import Jpgtobit
import matplotlib.pyplot as plt
import numpy as np
import math
import sys

from Node import Node
from algorithms.AStar import aStar
from algorithms.Prm import prm
from algorithms.Rrt import rrt


def doPathPlanning(gridraw, startcord, goalcord, planType, resolution=1):
    edgesize = int(len(gridraw) / resolution)
    gridNode = np.empty((edgesize, edgesize), Node)
    start = Node((int(startcord[0] / resolution), int(startcord[1] / resolution)),
                 value=0, edgeLength=resolution, descr='start')
    start.valuCalc(gridraw)
    goal = Node((int(goalcord[0] / resolution), int(goalcord[1] / resolution)),
                value=0, edgeLength=resolution, descr='goal')
    goal.valuCalc(gridraw)
    if start.value != 0 or goal.value != 0:
        raise Exception('Goal or start are in occupied cells')

    for x in range(edgesize):
        for y in range(edgesize):
            gridNode[x, y] = Node((x, y), value=0, edgeLength=resolution)
            gridNode[x, y].valuCalc(gridraw)

    gridNode[start.point[0], start.point[1]] = start
    gridNode[goal.point[0], goal.point[1]] = goal
    return planSelector(start, goal, gridNode, planType)


def planSelector(start, goal, gridNode, planType):
    if planType == 'aStar':
        return aStar(start, goal, gridNode)
    elif planType == 'prm':
        return prm(start, goal, gridNode)
    elif planType == 'rrt':
        return rrt(start, goal, gridNode)


def ObstacleScale1(grid, rr=0):
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if grid[x, y] == 1:
                for xx in range(-rr - 1, rr + 1):
                    for yy in range(-rr - 1, rr + 1):
                        try:
                            if grid[x + xx, y + yy] != 1:
                                grid[x + xx, y + yy] = 0.6
                        except:
                            continue
    grid = np.round(grid)
    return grid


def ObstacleScale(grid, rr=0):
    a = np.copy(grid)
    for i in range(rr):
        grid = grid + 0.001 * np.roll(a, i + 1, 0)  ##right
        grid = grid + 0.001 * np.roll(a, -(i + 1), 0)  ##left
        grid = grid + 0.001 * np.roll(a, i + 1, 1)  ##up
        grid = grid + 0.001 * np.roll(a, -(i + 1), 1)  ##down
        grid = grid + 0.001 * np.roll(np.roll(a, i + 1, 0), i + 1, 1)
        grid = grid + 0.001 * np.roll(np.roll(a, i + 1, 0), -(i + 1), 1)
        grid = grid + 0.001 * np.roll(np.roll(a, -(i + 1), 0), i + 1, 1)
        grid = grid + 0.001 * np.roll(np.roll(a, -(i + 1), 0), -(i + 1), 1)
    grid = np.ceil(grid * 1.01) / 2
    return grid


def gridSquare(grid):
    t = (math.log(len(grid), 2))
    t2 = 2 ** (int(t) + 1) - len(grid)
    sqgrid = np.vstack((grid, np.ones((t2, len(grid[0])))))
    t3 = abs(len(sqgrid) - len(sqgrid[0]))
    sqgrid = np.hstack((sqgrid, np.ones((len(sqgrid), t3))))
    return sqgrid


def roundToInt(val):
    return int(round(val))


start = AllConstants.start[0]
start_State = AllConstants.start
goal = AllConstants.goal[0]
goal_State = AllConstants.goal

importGrid = np.around(Jpgtobit.imgB_V2 * 1.01, decimals=0)
# rRadios = AllConstants.RobotRadios
rRadios = 20
plt.imshow(importGrid, cmap='hot', interpolation='nearest')
#plt.show()
obsScaleGrid = ObstacleScale(importGrid, rRadios)
plt.imshow(obsScaleGrid, cmap='hot', interpolation='nearest')
#plt.show()
importGrid = np.copy(obsScaleGrid)
# sqGrid = gridSquare(importGrid)
sqGrid = gridSquare(obsScaleGrid)
# sqGrid = obsScaleGrid
plt.imshow(sqGrid, cmap='hot', interpolation='nearest')
#plt.show()
"""
if sys.argv[1] == 'aStar':
    for p in range(int(math.log(len(sqGrid), 2)), 0, -1):
        try:
            path = doPathPlanning(sqGrid, start, goal, sys.argv[1], 2 ** p)
            break
        except Exception as e:
            print(e)
elif sys.argv[1] == 'prm':
    path = doPathPlanning(sqGrid, start, goal, sys.argv[1], 16)
"""
sum_ = 0
found = False
failure = 0
for j in range(1):
    importGrid = np.copy(obsScaleGrid)
    path = doPathPlanning(sqGrid, start, goal, sys.argv[1], AllConstants.RESOLUTION)
    if len(path) < 2:
        found = False
        failure += 1
    else:
        found = True

    pathNoScale = list()
    pathNoScale.append(start)
    for node in path:
        x, y = node.point
        res = node.edgeLength
        pathNoScale.append((roundToInt(x * res), roundToInt(y * res)))
        importGrid[roundToInt(x * res): roundToInt((x + 1) * res),
        roundToInt(y * res): roundToInt((y + 1) * res)] = 0.4

    importGrid[start[0]:start[0] + 5, start[1]:start[1] + 5] = 0.6
    importGrid[goal[0]:goal[0] + 5, goal[1]:goal[1] + 5] = 0.2
    pathNoScale.append(goal)

    total = 0
    for i in range(len(pathNoScale) - 1):
        total = total + AllConstants.dist(pathNoScale[i], pathNoScale[i + 1])
    if not found:
        total = 0
    print 'Path length =' + str(total)
    sum_ += total
    plt.imshow(importGrid[:, ::-1].transpose(), cmap='hot', interpolation='nearest')
    plt.show()
    #dir = 'results\misc\path_' + sys.argv[1] + '_' + str(j + 1) + '_' + str(total) + '.png'
    #plt.imsave(dir, importGrid[:, ::-1].transpose())

#print ('Avg=' + str(sum_ / (20 - failure)))
#print ("Faliure=" + str(failure))
print("END")
