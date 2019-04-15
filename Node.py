import numpy as np


class Node(object):
    def __init__(self, value, point, edgeLength=1, descr='Node'):
        self.value = value
        self.point = point
        self.edgeLength = edgeLength
        self.descr = descr
        self.parent = None
        self.H = 0
        self.G = 0
        self.neighbors = []
        self.id = -1

    def __str__(self):
        return str(self.point) + "," + str(self.G)

    def __cmp__(self, other):
        return self.priority - other.priority

    def valuCalc(self, gridraw):
        x, y = self.point
        e = self.edgeLength
        self.value = np.sum(gridraw[x * e: (x + 1) * e, y * e:(y + 1) * e])