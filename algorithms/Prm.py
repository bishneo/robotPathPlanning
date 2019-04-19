# Probablistic road map
import random
import math
import numpy as np
import scipy.spatial

import AllConstants
from Node import Node

show_animation = False


class KDTree:
    """
    Nearest neighbor search using KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, input, k=1):
        if len(input.shape) >= 2:
            index = []
            dist = []

            for i in input.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(input, k=k)
        return index, dist

    def searchInRadius(self, input, r):
        """
        Search for points with in a radius 'r'
        """

        index = self.tree.query_ball_point(input, r)
        return index


def prmPlanning(start, goal, obsX, obsY, rr):
    obsKDtree = KDTree(np.vstack((obsX, obsY)).T)

    sample_x, sample_y = getSamples(start.point, goal.point, rr, obsX, obsY, obsKDtree)
    road_map = generateRoadMap(sample_x, sample_y, rr, obsKDtree)
    path, rx, ry = dijkstraShortestPath(start, goal, road_map, sample_x, sample_y)

    return path, rx, ry


def isCollision(sx, sy, gx, gy, rr, obsKDTree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx ** 2 + dy ** 2)

    if d >= AllConstants.MAX_EDGE_LEN:
        return True

    D = rr
    nstep = int(round(d / D))

    for i in range(nstep):
        idxs, dist = obsKDTree.search(np.array([x, y]).reshape(2, 1))
        # check for collision
        if dist[0] <= rr:
            return True
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # check for goal
    idxs, dist = obsKDTree.search(np.array([gx, gy]).reshape(2, 1))
    if dist[0] <= rr:
        return True  # collision

    return False  # OK


def generateRoadMap(sample_x, sample_y, rr, obsKDTree):
    """
    Road map generation
    sample_x: x positions of the sampled points
    sample_y: y positions of the sampled points
    rr: Robot's radius
    obsKDtree: KDTree object of obstacles
    """

    road_map = []
    numSamples = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(numSamples), sample_x, sample_y):

        index, dists = skdtree.search(
            np.array([ix, iy]).reshape(2, 1), k=numSamples)
        inds = index[0]
        edge_id = []

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not isCollision(ix, iy, nx, ny, rr, obsKDTree):
                edge_id.append(inds[ii])
            else:
                print "Collision!"

            if len(edge_id) >= AllConstants.NUM_NEIGHBORS:
                break
        road_map.append(edge_id)
    return road_map


def dijkstraShortestPath(start, goal, roadMap, xSamples, ySamples):
    """
    start - start node
    goal - goal node
    roadMap - graph of all the sampled points.
    """
    nStart = start
    nGoal = goal
    path = []

    openset, closedset = dict(), dict()
    openset[len(roadMap) - 2] = nStart

    while True:
        if not openset:
            print("Cannot find path")
            break

        min_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[min_id]

        if min_id == (len(roadMap) - 1):
            print("goal is found!")
            nGoal.pId = current.pId
            nGoal.cost = current.cost
            break

        # Remove the item from the open set.
        del openset[min_id]
        # Add the item to closed set.
        closedset[min_id] = current

        for i in range(len(roadMap[min_id])):
            n_id = roadMap[min_id][i]
            dx = xSamples[n_id] - current.point[0]
            dy = ySamples[n_id] - current.point[1]
            d = math.sqrt(dx ** 2 + dy ** 2)
            node = Node(point=(xSamples[n_id], ySamples[n_id]), cost=current.cost + d, pId=min_id)

            # skip if node is in closed set.
            if n_id in closedset:
                continue

            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pId = min_id
            else:
                openset[n_id] = node

    # generate final course
    resX, resY = [nGoal.point[0]], [nGoal.point[1]]
    path.append(nGoal)
    pId = nGoal.pId
    while pId != -1:
        n = closedset[pId]
        resX.append(n.point[0])
        resY.append(n.point[1])
        pId = n.pId
        path.append(n)

    return path, resX, resY


def getSamples(start, goal, rr, obsX, obsY, obsKDtree):
    maxx = max(obsX)
    maxy = max(obsY)
    minx = min(obsX)
    miny = min(obsY)

    sample_x, sample_y = [], []
    while len(sample_x) <= AllConstants.N_SAMPLE:
        tx = (random.random() - minx) * (maxx - minx)
        ty = (random.random() - miny) * (maxy - miny)

        index, dist = obsKDtree.search(np.array([tx, ty]).reshape(2, 1))

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(start[0])
    sample_y.append(start[1])
    sample_x.append(goal[0])
    sample_y.append(goal[1])

    return sample_x, sample_y


def prm(start, goal, grid):
    obsX = []
    obsY = []

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j].value != 0:
                obsX.append(grid[i][j].point[0])
                obsY.append(grid[i][j].point[1])

    robotRadius = 1.5

    path, resX, resY = prmPlanning(start, goal, obsX, obsY, robotRadius)

    assert resX, 'Path not found'
    return path
