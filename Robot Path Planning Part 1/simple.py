import sys
import math
import AllConstants
import time
import ActionPlanner2
from collections import OrderedDict
import csv
from Atan import atan3


def blackBox(curState, desState, er):
    xchange = desState[0][0] - curState[0][0]
    ychange = desState[0][1] - curState[0][1]
    dist = math.sqrt(xchange ** 2 + ychange ** 2)
    if dist < er * 1.75:
        return 0, dist
    heading = math.atan2(ychange, xchange)
    rot_dist = heading - p.pa

    if abs(rot_dist) > math.radians(5):
        if rot_dist > 0:
            return 2, rot_dist
        else:
            return 3, rot_dist
    return 1, dist


start_time = time.time()

rRadios = AllConstants.RobotRadios
truep = ActionPlanner2.truep
path = ActionPlanner2.path_head
action = ActionPlanner2.actions
start = path[0]
goal = path[-1]
record = False

for i in range(len(path) - 2):
    xchange = path[i][0][0] - path[i + 1][0][0]
    ychange = path[i][0][1] - path[i + 1][0][1]
    heading1 = math.atan2(ychange, xchange)

    xchange = path[i + 1][0][0] - path[i + 2][0][0]
    ychange = path[i + 1][0][1] - path[i + 2][0][1]
    heading2 = math.atan2(ychange, xchange)

    if heading1 == heading2:
        path.pop(i+1)
    if path[i + 2][0][0] == goal[0][0] and path[i + 2][0][1] == goal[0][1]:
        break
print(path)

# ------------------------------------------------------------------------------------
# Check with "locate playerc.py"
sys.path.append('/usr/local/lib/python2.7/site-packages/')
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
sys.path.append('~/behrooz.mrd47@gmail.com/Wong/PathTracker/')
from playerc import *

# Create a client object
r = playerc_client(None, 'localhost', 6665)

# Connect it to the server
if r.connect() != 0:
    raise playerc_error_str()

# Create a proxy for position2d:0 device
p = playerc_position2d(r, 0)
if p.subscribe(PLAYERC_OPEN_MODE) != 0:
    raise playerc_error_str()

c = playerc_camera(r, 0)
if c.subscribe(PLAYERC_OPEN_MODE):
    raise Exception(playerc_error_str())

s = playerc_simulation(r, 0)
if s.subscribe(PLAYERC_OPEN_MODE):
    raise Exception(playerc_error_str())
s.set_pose2d("start", start[0][0], start[0][1], 5)

for i in range(len(path) - 1):
    name = "mile" + str(i)
    s.set_pose2d(name, path[i][0][0], path[i][0][1], 5)

s.set_pose2d("goal", goal[0][0], goal[0][1], 5)

#  /* read from the proxies */
if r.read() == None:
    raise playerc_error_str()
p.enable(True)
error = 7.0

for node in path:
    while abs(p.px - node[0][0]) > error or abs(p.py - node[0][1]) > error:
        if r.read() == None:
            raise playerc_error_str()

        print('Dest coord %.3f and cur heading:', node)
        print('Curr pose (%.3f,%.3f,%.3f):' % (p.px, p.py, p.pa))
        xchange = node[0][0] - p.px
        ychange = node[0][1] - p.py
        heading = math.atan2(ychange, xchange)
        roat_dist = abs(heading - p.pa)

        print(xchange, ychange, heading)
        while roat_dist > 0.015:
            yaw = abs(heading - p.pa) / (heading - p.pa) * 1
            # yaw = 0.1
            if abs(heading - p.pa) < 0.25:
                yaw = abs(heading - p.pa) / (heading - p.pa) * 0.1
                # yaw = 0.03
            p.set_cmd_vel(0, 0, yaw, 1)
            r.read()
            print('Dest heading to %.3f and cur heading %.3f:' % (heading, p.pa))
            roat_dist = abs(heading - p.pa)
        p.set_cmd_vel(0, 0, 0, 2)

        cord_dist = math.sqrt(xchange ** 2 + ychange ** 2)
        while cord_dist > error:
            speed = 20
            # if cord_dist < 3:
            #     speed = 2
            p.set_cmd_vel(speed, 0, 0, 3)
            r.read()
            print('heading to ', node, ' dist: ', cord_dist, 'cur speed: ', p.vx)
            cord_dist = math.sqrt((node[0][0] - p.px) ** 2 + (node[0][1] - p.py) ** 2)

p.set_cmd_vel(0, 0, 0, 4)

# archive = OrderedDict()
# counter = 0
# save_Freq = 1
# for node in path:
#     while abs(p.px - node[0][0]) > error or abs(p.py - node[0][1]) > error:
#         if record and counter % save_Freq == 0:
#             picName = 'Pic%d.ppm' % counter
#             c.save(picName)
#             a = {'pic': picName,
#                  'BB': blackBox(((p.px, p.py), p.pa), node, error),
#                  'CC': (0, 0),
#                  'dest': node,
#                  'pose': (p.px, p.py, p.pa)}
#             archive[counter] = a
#
#         print 'Dest coord:', node
#         print('Curr pose (%.3f,%.3f,%.3f):' % (p.px, p.py, p.pa))
#         xchange = node[0][0] - p.px
#         ychange = node[0][1] - p.py
#         heading = math.atan2(ychange, xchange)
#         roat_dist = heading - p.pa
#         print('xch: %f, ych: %f, head: %f' % (xchange, ychange, heading))
#         if abs(roat_dist) > math.radians(2):
#             yaw = abs(roat_dist) / roat_dist * 1
#             # yaw = 0.1
#             if abs(heading - p.pa) < 0.2:
#                 yaw = abs(roat_dist) / roat_dist * 0.1
#                 # yaw = 0.03
#         else:
#             yaw = 0
#
#         cord_dist = math.sqrt(xchange ** 2 + ychange ** 2)
#         if cord_dist > error:
#             speed = 15
#             if cord_dist < 6:
#                 speed = 3
#         else:
#             speed = 0
#         print('speed: ', speed, ' yaw: ', yaw)
#         p.set_cmd_vel(speed, 0, yaw, 1)
#
#         if record and counter % save_Freq == 0:
#             archive[counter]['CC'] = (speed, yaw)
#             print(str(archive[counter]))
#         r.read()
#         counter = counter + 1
#     p.set_cmd_vel(0, 0, 0, 4)

# Clean up (l.unsubcribe is for laser sensor)
p.unsubscribe()
r.disconnect()

with open('BB.txt', 'w') as file:
    wtr = csv.writer(file, dialect='excel', delimiter=';', lineterminator='\n')
    for k, v in archive.items():
        readItem = [k, 'picID:', v['pic'], 'BB:', v['BB'], 'CC:', v['CC'], 'dest:', v['dest'], 'pose:', v['pose']]
        wtr.writerow(readItem)
file.close()
