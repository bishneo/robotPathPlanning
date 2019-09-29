import math
import Astar
import AllConstants
import csv
from Atan import atan3


def moveToActionDiag(a, b):
    states = list()
    action = list()
    states.append(a)

    xchange = b[0][0] - states[-1][0][0]
    ychange = b[0][1] - states[-1][0][1]
    heading = atan3(ychange, xchange)
    roatation = heading - states[-1][1]
    while roatation != 0:
        if roatation > 0:
            action.append(2)
        else:
            action.append(3)
        states.append((states[-1][0], heading))
        roatation = heading - states[-1][1]

    dist = math.sqrt((states[-1][0][0] - b[0][0]) ** 2 + (states[-1][0][1] - b[0][1]) ** 2)
    while dist != 0:
        x_ch = math.trunc(math.cos(heading) * 1.5)
        y_ch = math.trunc(math.sin(heading) * 1.5)
        states.append(((states[-1][0][0] + x_ch, states[-1][0][1] + y_ch), heading))
        action.append(1)
        dist = math.sqrt((states[-1][0][0] - b[0][0]) ** 2 + (states[-1][0][1] - b[0][1]) ** 2)

    states.pop(0)
    return states, action


def moveToActionPerp(a, b):
    states = list()
    action = list()
    states.append(a)

    xchange = b[0][0] - states[-1][0][0]
    heading = 0
    if xchange < 0:
        heading = math.radians(180)
    roatation = heading - states[-1][1]

    while xchange != 0 and roatation != 0:
        if roatation > 0:
            action.append(2)
        else:
            action.append(3)
        states.append((states[-1][0], heading))
        roatation = heading - states[-1][1]

    while xchange != 0:
        x_ch = xchange / abs(xchange)
        states.append(((states[-1][0][0] + x_ch, states[-1][0][1]), heading))
        action.append(1)
        xchange = b[0][0] - states[-1][0][0]

    ychange = b[0][1] - states[-1][0][1]
    if ychange > 0:
        heading = math.radians(90)
    elif ychange < 0:
        heading = math.radians(270)
    roatation = heading - states[-1][1]

    while ychange != 0 and roatation != 0:
        if roatation > 0:
            action.append(2)
        else:
            action.append(3)
        states.append((states[-1][0], heading))
        roatation = heading - states[-1][1]

    while ychange != 0:
        y_ch = ychange / abs(ychange)
        states.append(((states[-1][0][0], states[-1][0][1] + y_ch), heading))
        action.append(1)
        ychange = b[0][1] - states[-1][0][1]

    states.pop(0)
    return states, action


path = Astar.pathNoScale
path_head = list()
for p in path:
    path_head.append((p, 1000))
path_head[0] = (AllConstants.start[0], math.radians(AllConstants.start[1]))
path_head[-1] = (AllConstants.goal[0], math.radians(AllConstants.goal[1]))

truep = list()
actions = list()
truep.append(path_head[0])
newcells, newactions = moveToActionPerp(truep[-1], path_head[1])
truep.extend(newcells)
actions.extend(newactions)

for i in range(2, len(path_head) - 1):
    newcells, newactions = moveToActionDiag(truep[-1], path_head[i])
    truep.extend(newcells)
    actions.extend(newactions)

newcells, newactions = moveToActionPerp(truep[-1], AllConstants.goal)
truep.extend(newcells)
actions.extend(newactions)

wtr = csv.writer(open('pathNoScale.txt', 'w'), delimiter=',', lineterminator='\n')
for x in path:
    wtr.writerow([x])

with open('TruePath.txt', 'w') as f:
    wtr = csv.writer(f, delimiter=',', lineterminator='\n')
    for i in range(len(actions)):
        wtr.writerow([truep[i], actions[i]])
    wtr.writerow([truep[-1], ])
f.close()
print("END2")
