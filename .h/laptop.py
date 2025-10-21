# ============================================================================
#                   *** FUNCTIONS FOR LAPTOP PROCESSING ***
# ============================================================================

# ==================================================
#                       OPEN CV
# ==================================================
import cv2                          # download this library plz :)
import numpy as np                  # download this library plz :)

# filters image - removes blur, background, and color
def removeNoise():

# finds contour lines in given image - looking for 4-vertices groups
def findContours():

# locate robot on whiteboard within xy coordinate system
# returns position of robot
def checkRobot():

# ==================================================
#                   PATH GENERATOR
# ==================================================
# takes in png/jpg image from laptop
# image data stored within program
def readImage():

# converts given png/jpg to svg
# returns array of vector paths (svg)
def convertToSvg():

# takes svg and sorts to create a path for robot
# returns sorted svg array
def sortPath():

# takes svg and generates GCode for robot
# sends a line of GCode to the robot (via bluetooth)
def generateGcode():