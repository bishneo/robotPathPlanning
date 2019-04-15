floorImgAdress = 'World.jpg'
RobotRadios = 10

actionDict = {(0, 0): 'stop', (1, 0): 'X+', (1, 1): 'X+Y+', (0, 1): 'Y+', (-1, 1): 'X-Y+',
              (-1, 0): 'X-',
              (-1, -1): 'X-Y-', (0, -1): 'Y-', (1, -1): 'X+Y-'}

start = ((80, 80), 90)
goal = ((432, 432), 90)

timeCons = 0.1  ##In seconds
Ang_Speed = 1  ##Rad per second
Forward_Speed = 10  ##meters per second
Error = 2  ##
