import copy
import math
import random
import matplotlib.pyplot as plt
import numpy as np


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, grid, expandDis=1.0, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start.point[0], start.point[1], start.value)
        self.goal = Node(goal.point[0], goal.point[1], goal.value)
        self.maxX = len(grid) - 1
        self.maxY = len(grid[0]) - 1
        self.grid = grid
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter

    def Planning(self, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rndX = np.random.choice(self.maxX)
                rndY = np.random.choice(self.maxY)
                tempNode = self.grid[int(round(rndX))][int(round(rndY))]
                rndNode = Node(tempNode.point[0], tempNode.point[1], tempNode.value)
            else:
                rndNode = self.goal

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rndNode)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rndNode.y - nearestNode.y, rndNode.x - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)

            newNode.x = int(round(newNode.x))
            newNode.y = int(round(newNode.y))
            tmp = self.grid[newNode.x][newNode.y]

            newNode.value = tmp.value
            newNode.point = (newNode.x, newNode.y)

            newNode.parent = nind

            if self.isCollision(newNode):
                continue

            # check goal
            dx = newNode.x - self.goal.x
            dy = newNode.y - self.goal.y
            d = math.sqrt(dx * dx + dy * dy)

            self.nodeList.append(newNode)
            #print("nNodelist:", len(self.nodeList))

            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph((rndNode.x, rndNode.y))

        path = [self.goal]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append(node)
            lastIndex = node.parent
        path.append(self.start)

        return path

    def DrawGraph(self, rnd=None):  # pragma: no cover
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 + (node.y - rnd.y)
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def isCollision(self, node):

        return node.value != 0


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, value):
        self.x = x
        self.y = y
        self.value = value
        self.point = (x, y)
        self.parent = None
        self.edgeLength = 16


def getObstacleList(grid):
    obstacleList = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j].value != 0:
                obstacleList.append(grid[i][j])
    return obstacleList


def rrt(start, goal, grid):
    obstacleList = getObstacleList(grid)
    rrt = RRT(start=start, goal=goal, grid=grid)
    path = rrt.Planning(animation=False)

    return path
