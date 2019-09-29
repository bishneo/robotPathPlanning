define INRobot_camera camera
(
    resolution [600 300]
	# Set the range so you don't see parts of the robot (or even the camera!)
	# in the camera
    range [3 200] # [min max]
    fov [120.0 60.0]
    pantilt [0 0] # + moves the cam to the left and down
    size [2 2 1]
    color "black"
    # watts 100
)
