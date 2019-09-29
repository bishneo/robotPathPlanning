import AllConstants
import Jpgtobit
import matplotlib.pyplot as plt
import numpy as np
import math


class Node(object):
    def __init__(self, value, point, edgeLength=1, descr='Node'):
        self.value = value
        self.point = point
        self.edgeLength = edgeLength
        self.descr = descr
        self.parent = None
        self.H = 0
        self.G = 0

    def __str__(self):
        return str(self.point) + "," + str(self.G)

    def __cmp__(self, other):
        return self.priority - other.priority

    def valuCalc(self, gridraw):
        x, y = self.point
        e = self.edgeLength
        self.value = np.sum(gridraw[x * e: (x + 1) * e, y * e:(y + 1) * e])


def children(node, grid):
    nodex, nodey = node.point
    links = []
    for x in range(3):
        for y in range(3):
            try:
                if grid[nodex + x - 1, nodey + y - 1].value == 0:
                    links.append(grid[nodex + x - 1, nodey + y - 1])
            except:
                continue
    # links = [grid[d[0]][d[1]] for d in [(x - 1, y), (x, y - 1), (x, y + 1), (x + 1, y)]]
    # return [link for link in links if link.value == 0]
    return links


def h1func(node1, node2):
    return abs(node1.point[0] - node2.point[0]) + abs(node1.point[1] - node2.point[1])


def h2func(node1, node2):
    return math.sqrt((node1.point[0] - node2.point[0]) ** 2 + (node1.point[1] - node2.point[1]) ** 2)


def aStar(start, goal, grid):
    # The open and closed sets
    openset = set()
    closedset = set()
    # Current point is the starting point
    current = start
    # Add the starting point to the open set
    openset.add(current)
    # While the open set is not empty
    while openset:
        # Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o: o.G + o.H)
        # If it is the item we want, retrace the path and return it
        if current.descr == 'goal':
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            print('Path is found')
            return path[::-1]
        # Remove the item from the open set
        openset.remove(current)
        # Add it to the closed set
        closedset.add(current)
        # Loop through the node's children/siblings
        for node in children(current, grid):
            # If it is already in the closed set, skip it
            if node in closedset:
                continue
            # Otherwise if it is already in the open set
            if node in openset:
                # Check if we beat the G score
                new_g = current.G + 1
                new_h = h2func(node, goal)
                if node.G + node.H > new_g + new_h:
                    # If so, update the node to have a new parent
                    node.G = new_g
                    node.H = new_h
                    node.parent = current
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + 1
                node.H = h2func(node, goal)
                # Set the parent to our current item
                node.parent = current
                # Add it to the set
                openset.add(node)
    # Throw an exception if there is no path
    raise ValueError('No Path Found')


def perfomrAstar(gridraw, startcord, goalcord, resolution=1):
    edgesize = int(len(gridraw) / resolution)
    emptyNode = Node(0, (0, 0), resolution)
    gridNode = np.empty((edgesize, edgesize), Node)

    startx, starty = startcord
    goalx, goaly = goalcord
    start = Node(0, (int(startcord[0] / resolution), int(startcord[1] / resolution)), resolution, 'start')
    start.valuCalc(gridraw)
    goal = Node(0, (int(goalcord[0] / resolution), int(goalcord[1] / resolution)), resolution, 'goal')
    goal.valuCalc(gridraw)
    if start.value != 0 or goal.value != 0:
        raise Exception('Goal or start are in occupied cells')

    for x in range(edgesize):
        for y in range(edgesize):
            gridNode[x, y] = Node(0, (x, y), resolution)
            gridNode[x, y].valuCalc(gridraw)

    gridNode[start.point[0], start.point[1]] = start
    gridNode[goal.point[0], goal.point[1]] = goal
    return aStar(start, goal, gridNode)


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


start = AllConstants.start[0]
start_State = AllConstants.start
goal = AllConstants.goal[0]
goal_State = AllConstants.goal

importGrid = np.around(Jpgtobit.imgB_V2 * 1.01, decimals=0)
rRadios = AllConstants.RobotRadios
plt.imshow(importGrid, cmap='hot', interpolation='nearest')
# plt.show()
obsScaleGrid = ObstacleScale(importGrid, rRadios)
plt.imshow(obsScaleGrid, cmap='hot', interpolation='nearest')
# plt.show()
importGrid = np.copy(obsScaleGrid)
sqGrid = gridSquare(obsScaleGrid)
plt.imshow(sqGrid, cmap='hot', interpolation='nearest')
# plt.show()

path = list()
for p in range(int(math.log(len(sqGrid), 2)), 0, -1):
    try:
        p = 3
        path = perfomrAstar(sqGrid, start, goal, 2 ** p)
        break
    except Exception as e:
        print(e)

pathNoScale = list()
pathNoScale.append(start)
for node in path:
    x, y = node.point
    res = node.edgeLength
    pathNoScale.append((x * res, y * res))
    importGrid[x * res: (x + 1) * res, y * res: (y + 1) * res] = 0.4
importGrid[start[0] - 5:start[0] + 6, start[1] - 5:start[1] + 6] = 0.6
importGrid[goal[0] - 5:goal[0] + 6, goal[1] - 5:goal[1] + 6] = 0.2
pathNoScale.append(goal)

plt.imshow(importGrid[:, ::-1].transpose(), cmap='hot', interpolation='nearest')
plt.show()

from AllConstants import dist
total = 0
for i in range(len(pathNoScale)-1):
    total = total + dist(pathNoScale[i],pathNoScale[i+1])
print total

print("END A*")
