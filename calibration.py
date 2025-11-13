import numpy as np
import cv2 as cv
import glob

# --- 1. Define the ChArUco Board Parameters ---
# These MUST match the board you printed
SQUARES_VERTICALLY = 8
SQUARES_HORIZONTALLY = 11
SQUARE_LENGTH = 0.015 # typically in meters or whatever unit you choose
MARKER_LENGTH = 0.011 # Must be in the same unit as SQUARE_LENGTH

# --- 2. Setup the Dictionary and Board Object ---
# Use the dictionary corresponding to your board (e.g., DICT_5X5_100)
ARUCO_DICT = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50) # Example dictionary
BOARD = cv.aruco.CharucoBoard((SQUARES_HORIZONTALLY, SQUARES_VERTICALLY), SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT)
# If using a very old OpenCV version, you might need:
# BOARD = cv.aruco.CharucoBoard_create(SQUARES_HORIZONTALLY, SQUARES_VERTICALLY, SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT)

# --- 3. Setup Detector Parameters (Revised for compatibility) ---

# Use CharucoParameters for the CharucoDetector constructor
charuco_params = cv.aruco.CharucoParameters() 

# You can also use DetectorParameters() for general ArUco detection settings if needed
# detector_params = cv.aruco.DetectorParameters() 

try:
    # Try initializing with the specific CharucoParameters object
    detector = cv.aruco.CharucoDetector(BOARD, charucoParams=charuco_params)
    
except TypeError:
    # Fallback for older versions that might combine both parameters objects implicitly
    print("Falling back to standard DetectorParameters initialization.")
    detector_params = cv.aruco.DetectorParameters()
    detector = cv.aruco.CharucoDetector(BOARD, detector_params) # Try using the general one as a fallback

# Arrays to store points for calibration
allCharucoCorners = [] # Corners in image plane
allCharucoIds = []     # Ids of the corners
# You also need to keep track of the shape of the images
img_shapes = []

images = glob.glob(r'C:\Users\snovyang\eecs473\images\*.jpg') 

for fname in images:
    img = cv.imread(fname)
    if img is None:
        print(f"Warning: Could not read image {fname}")
        continue
        
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img_shapes.append(gray.shape[::-1]) # Store image width and height

    # --- Use the ChArUco detector ---
    # detectBoard returns charucoCorners and charucoIds
    charucoCorners, charucoIds, _, _ = detector.detectBoard(gray)
    
    if charucoIds is not None and len(charucoIds) > 0:
        print(f"ChArUco corners found in {fname}! Detected {len(charucoIds)} corners.")
        allCharucoCorners.append(charucoCorners)
        allCharucoIds.append(charucoIds)
        
        # Optional: Draw the corners to verify
        cv.aruco.drawDetectedCornersCharuco(img, charucoCorners, charucoIds)
        cv.imshow('ChArUco Detection', img)
        cv.waitKey(500)
    else:
        print(f"Warning: No ChArUco pattern found in {fname}.")

cv.destroyAllWindows()

# --- 4. Calibrate the camera using the ChArUco specific function ---

if len(allCharucoCorners) > 0:
    print(f"Starting camera calibration with {len(allCharucoCorners)} valid images.")
    
    # Note: ChArUco calibration uses the general cv.calibrateCamera function in newer APIs, 
    # but the point handling is different than a standard chessboard.
    # The structure below follows the standard calibrateCamera() format, which works if you accumulate points correctly:

    # The issue is that standard calibrateCamera needs 3D object points generated per image.
    # A cleaner approach uses the dedicated function for ChArUco:
    
    # Use cv.aruco.calibrateCameraCharucoExtended or the simplified cv.calibrateCamera if using 4.7.0+
    
    # This requires converting the ChArUco points into the format expected by calibrateCamera
    # For simplicity, here is the function call:

    # The function expects objpoints and imgpoints lists like the standard calibration script
    # ChArUco handles this internally when using the correct functions.
    # A common approach is to use the dedicated `calibrateCameraCharuco` function from `opencv-contrib-python`.
    # Make sure you have `pip install opencv-contrib-python`

    try:
        # This function is available in the contrib module and simplifies the process
        ret, mtx, dist, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
            allCharucoCorners, allCharucoIds, BOARD, gray.shape[::-1], None, None
        )
        print(f"Calibration successful! Reprojection error: {ret}")

        # ... (Your existing undistortion code can go here) ...
        # img = cv.imread('callibration (1).jpg')
        # ... 

    except AttributeError:
        print("Error: `cv.aruco.calibrateCameraCharuco` function not found. Ensure you have `opencv-contrib-python` installed and are using a compatible version.")

else:
    print("FATAL ERROR: Calibration failed because no valid ChArUco patterns were detected in any image.")
