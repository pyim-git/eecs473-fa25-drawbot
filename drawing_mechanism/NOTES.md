==================
## GANTRY CLASS
==================
#### DONE FUNCTIONS
* init() - initialize gantry class variable - must call before using gantry class variable
* moveRight(distance, speed) - moves gantry arm to the right by specified mm in mm/s
* moveLeft(distance, speed) - moves gantry arm to the left by specified mm in mm/s
* grab() - grips marker (tighten grip on marker)
* release() - let's go of marker (loosen grip on marker)
* markerDown() - rotates z-axis servo down and lowers marker (use when wanting to draw)
* markerUp() - rotates z-axis servo up and raises marker (use when done drawing)
* putMarkerBack() - puts marker in gripper back into tool changing rack
* takeMarkerFrom() - takes marker from the tool changing rack when no marker is in gripper
==================
#### NEED-TO-FINISH FUNCTIONS
* tweaking tool changing functions to be better for users