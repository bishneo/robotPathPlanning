import copy
import math
import random
import numpy as np

import AllConstants
from Node import Node


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, grid, expandDistance=1.0, goalSampleRate=5):
        """
        start node
        goal node
        grid world
        distance to expand by
        number of times goal is selected as sample
        """
        self.start = start
        self.goal = goal
        self.maxX = len(grid) - 1
        self.maxY = len(grid[0]) - 1
        self.grid = grid
        self.expandDistance = expandDistance
        self.goalSampleRate = goalSampleRate

    def rrtPlanning(self):
        count = 0
        found = False
        self.nodeList = [self.start]
        while True:
            # Sample randomly
            if random.randint(0, 100) > self.goalSampleRate:
                rndX = np.random.choice(self.maxX)
                rndY = np.random.choice(self.maxY)
                tempNode = self.grid[int(round(rndX))][int(round(rndY))]
                rndNode = Node(tempNode.point, value=tempNode.value)
            else:
                rndNode = self.goal

            # Find nearest node
            nind = self.getIndexOfNearestNode(self.nodeList, rndNode)

            # calculate angle to sample node.
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rndNode.point[1] - nearestNode.point[1],
                               rndNode.point[0] - nearestNode.point[0])

            newNode = copy.deepcopy(nearestNode)

            # Calculate coordinates of the next node.
            p = list(newNode.point)
            p[0] += self.expandDistance * math.cos(theta)
            p[1] += self.expandDistance * math.sin(theta)

            p[0] = int(round(p[0]))
            p[1] = int(round(p[1]))
            tmp = self.grid[p[0]][p[1]]

            newNode.value = tmp.value
            newNode.point = tuple(p)

            newNode.parent = nind

            if self.isCollision(newNode):
                continue

            # calculate distance from goal
            dx = newNode.point[0] - self.goal.point[0]
            dy = newNode.point[1] - self.goal.point[1]
            d = math.sqrt(dx * dx + dy * dy)

            self.nodeList.append(newNode)

            if d <= self.expandDistance:
                found = True
                break
            if count > AllConstants.LIMIT:
                found = False
                break

            count += 1

        path = [self.goal]
        if found:
            print ("Path found!")
            lastIndex = len(self.nodeList) - 1
            while self.nodeList[lastIndex].parent is not None:
                node = self.nodeList[lastIndex]
                path.append(node)
                lastIndex = node.parent
        else:
            print ("Path not found!")
        path.append(self.start)

        return path

    def getIndexOfNearestNode(self, nodeList, rnd):
        distList = [(node.point[0] - rnd.point[0]) ** 2 + (node.point[1] - rnd.point[1])
                    ** 2 for node in nodeList]
        minind = distList.index(min(distList))
        return minind

    def isCollision(self, node):
        return node.value != 0


def rrt(start, goal, grid):
    rrt = RRT(start=start, goal=goal, grid=grid)
    path = rrt.rrtPlanning()

    return path
