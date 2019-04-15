import math

import numpy as np


def getRandomSamplePoints(grid, numNodes=1000):
    randomSamples = set()
    while (numNodes > len(randomSamples)):
        randX = np.random.choice(64)
        randY = np.random.choice(64)
        p = grid[randX, randY]

        if p.value == 0:
            randomSamples.add(p)

    return randomSamples


def isSamePoint(p1, p2):
    return p1[0] == p2[0] and p1[1] == p2[1]


def isIntersectObs(p1, p2, grid):
    return False


def getEuclideanDistance(p1, p2):
    return math.sqrt(math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2))


def generateRoadMap(samples, grid):
    for s in samples:
        distanceMap = []
        for n in samples:
            if not isSamePoint(s.point, n.point) and not isIntersectObs(s.point, n.point, grid):
                distanceMap.append((getEuclideanDistance(s.point, n.point), n))

        distanceMap = sorted(distanceMap, key=lambda x: x[0])
        count = 0
        for pair in distanceMap:
            if (count >= 10):
                break
            s.neighbors.append(pair[1])
            count += 1
    return samples


def generateNodeIds(samples, start, goal):
    start.id = 0
    goal.id = len(samples) + 1

    count = 1
    for s in samples:
        s.id = count
        count += 1

    samples = list(samples)
    samples.append(start)
    samples.append(goal)

    return samples


def getShortestPath(samples):
    nodeMap = {}

    for s in samples:
        nodeMap[s.id] = s

    numNodes = len(samples)
    dist = [10000.0] * (numNodes)
    vset = [True] * (numNodes)
    prev = [-1] * (numNodes)

    dist[0] = 0

    while True:
        if (sum(vset) == 0):
            break

        low = 10000.0
        u = -1
        for i in xrange(numNodes):
            if (vset[i]):
                if (u == -1 or dist[i] < low):
                    low = dist[i]
                    u = i

        vset[u] = False

        for v in nodeMap[u].neighbors:

            alt = dist[u] + getEuclideanDistance(nodeMap[u].point, v.point)

            if (alt < dist[v.id]):
                dist[v.id] = alt
                prev[v.id] = u

    node = 1
    path = []
    while True:
        if (node == -1):
            break
        path.append(node)
        node = prev[node]

    print path
    path_nodes = []
    for p in path:
        path_nodes.append(nodeMap[p])
    print path_nodes
    return path_nodes


def prm(start, goal, grid):
    samples = getRandomSamplePoints(grid, 1000)
    samples = generateNodeIds(samples, start, goal)
    roadMap = generateRoadMap(samples, grid)
    path = getShortestPath(roadMap)
    print "prm in progress"
    return path
