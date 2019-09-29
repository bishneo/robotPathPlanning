import math

worldImg = 1
floorImgAdress = 'Slide' + str(worldImg) + '.JPG'
RobotRadios = 20

actionDict = {(0, 0): 'stop', (1, 0): 'X+', (1, 1): 'X+Y+', (0, 1): 'Y+', (-1, 1): 'X-Y+', (-1, 0): 'X-',
              (-1, -1): 'X-Y-', (0, -1): 'Y-', (1, -1): 'X+Y-'}

if floorImgAdress == 'Slide1.JPG':
    start = ((100, 760), 270)
    goal = ((600, 360), 270)
if floorImgAdress == 'Slide2.JPG':
    start = ((600, 500), 90)
    goal = ((200, 800), 90)
if floorImgAdress == 'Slide3.JPG':
    start = ((80, 80), 90)
    goal = ((800, 80), 90)
if floorImgAdress == 'Slide4.JPG':  ##Blocked map
    start = ((80, 80), 90)
    goal = ((800, 432), 90)
# start = ((200, 100), 270)
# # start = ((830, 700), 270)
# # start = ((920, 60), 270)
# # start = ((200, 100), 270)
# # goal = ((600, 360), 270)
# goal = ((440, 400), 90)

timeCons = 0.1  ##In seconds
Ang_Speed = 1  ##Rad per second
Forward_Speed = 10  ##meters per second
Error = 2  ##

Init_Vel = [0.0, 0.0]  ##Initial velocity in speed and omega


def dist(x, y):
    return math.sqrt((y[0] - x[0]) ** 2 + (y[1] - x[1]) ** 2)
