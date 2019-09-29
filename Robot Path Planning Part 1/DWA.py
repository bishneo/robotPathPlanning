import math
import numpy as np
import time

import matplotlib.pyplot as plt
import sys

import AllConstants
from AllConstants import dist
import Astar
from Atan import atan3

sys.path.append("../../")
range_deg = [0, 45, 90, 135, 180, 225, 270, 315, 360]
range_rad = np.multiply(range_deg, (math.pi / 180))


class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_speed = 5  # [m/s]
        self.min_speed = -1  # [m/s]
        self.max_yawrate = 0.3  # [rad/s]
        self.max_accel = 1  # [m/ss]
        self.max_dyawrate = 0.1  # [rad/ss]
        self.v_reso = 0.5  # [m/s]
        self.yawrate_reso = 0.02  # [rad/s]
        self.dt = 0.5  # [s]
        self.predict_time = 5  # [s]
        self.to_goal_cost_gain = 0.5
        self.speed_cost_gain = 1.0
        self.robot_radius = AllConstants.RobotRadios  # [m]


def motion(x, u, dt):
    # motion model

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_trajectory(xinit, v, y, config):
    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def closest(x, ob):
    xx = x[0]
    yy = x[1]

    minDist = float("inf")
    found_Obs = (-1, -1)

    for a in range_rad:
        x_ch = math.trunc(math.cos(a) * 1.5)
        y_ch = math.trunc(math.sin(a) * 1.5)
        r = math.sqrt(math.sin(a) ** 2 + math.cos(a) ** 2)
        for d in range(1, 200, 7):
            x2 = int(xx) + d * x_ch
            y2 = int(yy) + d * y_ch
            if ob[x2, y2] == 1:
                distObs = dist((xx, yy), (x2, y2))
                if distObs < minDist:
                    minDist = distObs
                    found_Obs = (x2, y2)
                break

    return minDist, found_Obs


def calc_final_input(x, u, dw, config, goal, ob):
    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])

    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1] + config.v_reso, config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, config)

            # calc cost
            to_goal_cost = 5.5 * calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)
            # print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # print (final_cost)

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    return min_u, best_traj


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 3
    xx = traj[0, 0]
    yy = traj[0, 1]
    minr, obs = closest((xx, yy), ob)

    for ii in range(1, len(traj[:, 1]), skip_n):

        xx2 = traj[ii, 0]
        yy2 = traj[ii, 1]

        if int(xx2) != int(xx) or int(yy2) != int(yy):
            xx = xx2
            yy = yy2
            if ob[int(xx2), int(yy2)] > 0:
                return float("Inf")

            dist, obs = closest((xx2, yy2), ob)
            if dist < minr:
                minr = dist

        # if xx2 != traj[ii - 1, 0] or yy2 != traj[ii - 1, 0]:
        #     dist, obs = closest((xx2, yy2), ob)
        #     if dist < minr:
        #         minr = dist

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    xchange = goal[0] - traj[-1, 0]
    ychange = goal[1] - traj[-1, 1]
    heading = atan3(ychange, xchange)
    if heading == math.pi *2:
        heading = 0
    roat_dist = heading - traj[-1, 2]
    cost = config.to_goal_cost_gain * abs(roat_dist)

    # goal_magnitude = math.sqrt(goal[0] ** 2 + goal[1] ** 2)
    # traj_magnitude = math.sqrt(traj[-1, 0] ** 2 + traj[-1, 1] ** 2)
    # dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
    # error = dot_product / (goal_magnitude * traj_magnitude)
    # error_angle = math.acos(error)
    # cost = config.to_goal_cost_gain * error_angle

    return cost


def dwa_control(x, u, config, goal, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u, traj = calc_final_input(x, u, dw, config, goal, ob)

    return u, traj


start_time = time.time()

start = AllConstants.start[0]
start_State = AllConstants.start
goal = AllConstants.goal[0]
goal_State = AllConstants.goal

importGrid_DWA = np.copy(Astar.obsScaleGrid)
sqGrid = np.copy(Astar.obsScaleGrid)
plt.imshow(sqGrid, cmap='hot', interpolation='nearest')
# plt.show()

x = np.array([start_State[0][0], start_State[0][1], math.radians(start_State[1]), 0.0, 0.0])
# goal position [x(m), y(m)]
goal = np.array(AllConstants.goal[0])

u = np.array(AllConstants.Init_Vel)
config = Config()
traj = np.array(x)
traj = np.vstack((traj, x))

for i in range(1000):

    u, ltraj = dwa_control(x, u, config, goal, importGrid_DWA)
    x = motion(x, u, config.dt)
    traj = np.vstack((traj, x))  # store state history

    if dist(traj[-1][0:2], goal) < AllConstants.RobotRadios:
        print "!!!!!!!!!!END DWA"
        break

# end_Time = time.time()
# print end_Time - start_time

for i in range(len(traj)):
    importGrid_DWA[int(traj[i, 0]), int(traj[i, 1])] = i / 10

for i in range(len(traj)):
    x = traj[i, 0]
    y = traj[i, 1]
    importGrid_DWA[x - 3:x + 4, y - 3:y + 4] = 0.4
importGrid_DWA[start[0] - 5:start[0] + 6, start[1] - 5:start[1] + 6] = 0.6
importGrid_DWA[goal[0] - 5:goal[0] + 6, goal[1] - 5:goal[1] + 6] = 0.2

plt.imshow(importGrid_DWA[:, ::-1].transpose(), cmap='hot', interpolation='nearest')
plt.show()

from AllConstants import dist
total = 0
for i in range(len(traj) - 1):
    total = total + dist(traj[i, 0:2], traj[i + 1, 0:2])
print total

print "END DWA"