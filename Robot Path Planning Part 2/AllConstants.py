import math

worldImg = 1
floorImgAdress = 'worlds/world' + worldImg.__str__() + '.jpg'
RobotRadios = 10
RESOLUTION = 8

actionDict = {(0, 0): 'stop', (1, 0): 'X+', (1, 1): 'X+Y+', (0, 1): 'Y+', (-1, 1): 'X-Y+',
              (-1, 0): 'X-',
              (-1, -1): 'X-Y-', (0, -1): 'Y-', (1, -1): 'X+Y-'}
# Start and goal for different worlds.
if worldImg == 1:
    start = ((100, 760), 270)
    goal = ((600, 360), 270)
if worldImg == 2:
    start = ((600, 500), 90)
    goal = ((850, 500), 90)
if worldImg == 3:
    start = ((80, 80), 90)
    goal = ((800, 80), 90)
if worldImg == 4:
    start = ((80, 80), 90)
    goal = ((800, 432), 90)

N_SAMPLE = 2000 * 16 / RESOLUTION  # number of sample_points
NUM_NEIGHBORS = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 10.0  # [m] Maximum edge length
timeCons = 0.1  ##In seconds
Ang_Speed = 1  ##Rad per second
Forward_Speed = 10  ##meters per second
Error = 2  ##
LIMIT = 10000 * 16 / RESOLUTION


def dist(x, y):
    return math.sqrt((y[0] - x[0]) ** 2 + (y[1] - x[1]) ** 2)
