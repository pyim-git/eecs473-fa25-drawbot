import cv2
import cv2.aruco as aruco
import numpy as np


# Load your calibration
calib_data = np.load('backend/camera_calibration.npz')
camera_matrix = calib_data['camera_matrix']
dist_coeffs = calib_data['dist_coeffs']

# distinct IDs for the 4 whiteboard tags and the robot aruco tag
BOARD_IDS = [5,1,6,3]
ROBOT_ID = 7

robot_position = [-1, -1]

def detect_tags(frame, corners, ids, rvecs, tvecs):
    """detects and stores tag positions"""
    robot_tag = None
    board_tags = []
    
    # Separate robot tag from fixed tags
    for i, tag_id in enumerate(ids.flatten()):
        if tag_id == ROBOT_ID:
            robot_tag = {
                'position': tvecs[i][0].copy(),
                'rotation': rvecs[i][0].copy(),
                'corners': corners[i]
            }
        elif tag_id in BOARD_IDS:
            board_tags.append({
                'id': tag_id,
                'position': tvecs[i][0].copy(),
                'rotation': rvecs[i][0].copy(),
                'corners': corners[i]
            })
    
    return robot_tag, board_tags



def find_pos(robot_tag, board_tags):
    """Calculate robot position in global coordinates using homography"""

    if len(board_tags) < 4:
        print("Need 4 whiteboard tags for absolute positioning")
        return None
    
    try:
        # Get corresponding points between image and global coordinates
        image_points = []
        global_points = []
        
        for board_tag in board_tags:
            tag_id = board_tag['id']
            if tag_id in global_positions:
                # Use the center of the tag as reference point
                center = np.mean(board_tag['corners'][0], axis=0)
                image_points.append(center)
                global_points.append(global_positions[tag_id])  
        
        # Add robot position
        robot_center = np.mean(robot_tag['corners'][0], axis=0)
        image_points.append(robot_center)
        
        image_points = np.array(image_points, dtype=np.float32)
        global_points = np.array(global_points, dtype=np.float32)
        
        # Calculate homography matrix
        H, mask = cv2.findHomography(image_points[:-1], global_points, cv2.RANSAC, ransacReprojThreshold=1.5, confidence =0.999)
        
        if H is not None:
            # Transform robot position to global coordinates
            robot_img_point = np.array([image_points[-1][0], image_points[-1][1], 1.0])
            robot_world_homogeneous = H @ robot_img_point
            robot_world = robot_world_homogeneous / robot_world_homogeneous[2]
            
            return robot_world[:2]
    
    except Exception as e:
        print(f"Error in homography calculation: {str(e)}")
        return None
    
    return None


def get_pos(aruco_dict, parameters, cap):

    # Read a frame from the webcam
    ret, frame = cap.read()

    if not ret: return None, None

    # Convert the frame to grayscale for marker detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the ArUco markers in the grayscale frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    robot_pos = None

    if ids is not None: 
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        marker_size = 0.05  # size of tag in METERS
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        # Detect robot and whiteboard tags
        robot_tag, board_tags = detect_tags(frame, corners, ids, rvecs, tvecs)

        # did the camera detect all five tags? 
        if robot_tag is not None and len(board_tags) >= 4:
            robot_pos = find_pos(robot_tag, board_tags)

            if robot_pos is not None:        
                # Draw coordinate axes for robot
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                                robot_tag['rotation'], robot_tag['position'], marker_size * 0.5)
                
                # Draw coordinate axes for board tags
                for board_tag in board_tags:
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                                    board_tag['rotation'], board_tag['position'], marker_size * 0.5)
        else:
            # Show detection status
            detected_board = len(board_tags) if 'board_tags' in locals() else 0
            robot_detected = "YES" if robot_tag is not None else "NO"
            cv2.putText(frame, f"Board tags: {detected_board}/4 | Robot: {robot_detected}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            
    return robot_pos, frame


if __name__ == "__main__":
     # Define the ArUco dictionary to use
    # You can choose different dictionaries like DICT_4X4_50, DICT_5X5_100, etc.
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    # Define the ArUco detection parameters
    parameters = aruco.DetectorParameters()

    # Open the Logitech webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    while True:
        robot_pos, frame = get_pos(aruco_dict, parameters, cap)

        if robot_pos is not None:
            x_mm, y_mm = robot_pos
            print(f"ROBOT_POS:{x_mm:.1f},{y_mm:.1f}")

        # Break the loop if 'q' is pressed
        cv2.imshow('ArUco Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # Release the webcam and destroy all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    
def generate_frames(width_mm=1500, height_mm=1500):
    """
    Generator function that yields processed video frames with ArUco detection.
    This is used for video streaming to the web interface.
    """
    global robot_position
    # Define known (x,y) positions of whiteboard tags in (millimeters)
    global_positions = {
        BOARD_IDS[0]: [0, 0],                # top-left corner (origin)
        BOARD_IDS[1]: [0, height_mm],        # Bottom-left corner 
        BOARD_IDS[2]: [width_mm, 0],         # Top-right corner
        BOARD_IDS[3]: [width_mm, height_mm]  # Bottom-right corner
    }
    # Define the ArUco dictionary to use
    # You can choose different dictionaries like DICT_4X4_50, DICT_5X5_100, etc.
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    # Define the ArUco detection parameters
    parameters = aruco.DetectorParameters()

    # Open the Logitech webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    while True:
        robot_pos, frame = get_pos(aruco_dict, parameters, cap)
        if robot_pos is not None:
            x_mm, y_mm = robot_pos
            robot_position = robot_pos
            print(f"ROBOT_POS:{x_mm:.1f},{y_mm:.1f}")
        if frame is None:
            continue
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
        
        # Yield the frame in byte format
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

def get_robo_loc():
    x, y = robot_position
    return {"x": x, "y": y}

if __name__ == "__main__":
    detect_aruco_from_webcam()