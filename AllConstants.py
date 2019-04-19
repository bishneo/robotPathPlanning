worldImg = 2
floorImgAdress = 'worlds/world' + worldImg.__str__() + '.jpg'
RobotRadios = 10
RESOLUTION=8

actionDict = {(0, 0): 'stop', (1, 0): 'X+', (1, 1): 'X+Y+', (0, 1): 'Y+', (-1, 1): 'X-Y+',
              (-1, 0): 'X-',
              (-1, -1): 'X-Y-', (0, -1): 'Y-', (1, -1): 'X+Y-'}
# Start and goal for different worlds.
if worldImg == 1:
    start = ((80, 80), 90)
    goal = ((432, 432), 90)
if worldImg == 2:
    start = ((80, 80), 90)
    goal = ((800, 432), 90)
if worldImg == 3:
    start = ((80, 80), 90)
    goal = ((800, 432), 90)
if worldImg == 4:
    start = ((80, 80), 90)
    goal = ((800, 432), 90)

N_SAMPLE = 600*16/RESOLUTION  # number of sample_points
NUM_NEIGHBORS = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 10.0  # [m] Maximum edge length
timeCons = 0.1  ##In seconds
Ang_Speed = 1  ##Rad per second
Forward_Speed = 10  ##meters per second
Error = 2  ##
LIMIT = 1000*16/RESOLUTION