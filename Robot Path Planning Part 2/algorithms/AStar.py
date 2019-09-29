import math
import numpy as np


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
    return math.sqrt(
        (node1.point[0] - node2.point[0]) ** 2 + (node1.point[1] - node2.point[1]) ** 2)


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
