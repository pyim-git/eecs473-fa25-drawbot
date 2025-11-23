import cv2
import cv2.aruco as aruco

# Global video capture object
_cap = None

def get_video_capture():
    """Get or create the video capture object"""
    global _cap
    if _cap is None or not _cap.isOpened():
        _cap = cv2.VideoCapture(1)
        if not _cap.isOpened():
            # Try default camera if camera 1 fails
            _cap = cv2.VideoCapture(0)
    return _cap

def release_video_capture():
    """Release the video capture object"""
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None

def detect_aruco_from_webcam():
    """Original function for standalone use"""
    # Define the ArUco dictionary to use
    # You can choose different dictionaries like DICT_4X4_50, DICT_5X5_100, etc.
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

    # Define the ArUco detection parameters
    parameters = aruco.DetectorParameters()

    # Open the default webcam
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Convert the frame to grayscale for marker detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the ArUco markers in the grayscale frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If markers are detected, draw their boundaries and IDs
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, corner_set in enumerate(corners):
                for corner in corner_set[0]:
                    x, y = int(corner[0]), int(corner[1])
                    cv2.circle(frame, (x, y), 3, (0, 255, 0), -1) # Draw a green circle at each corner

        # Display the frame with detected markers
        cv2.imshow('ArUco Detection', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and destroy all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

def generate_frames():
    """
    Generator function that yields processed video frames with ArUco detection.
    This is used for video streaming to the web interface.
    """
    # Define the ArUco dictionary to use
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    
    # Define the ArUco detection parameters
    parameters = aruco.DetectorParameters()
    
    # Get video capture
    cap = get_video_capture()
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Could not read frame.")
            break
        
        # Convert the frame to grayscale for marker detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect the ArUco markers in the grayscale frame
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # If markers are detected, draw their boundaries and IDs
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, corner_set in enumerate(corners):
                for corner in corner_set[0]:
                    x, y = int(corner[0]), int(corner[1])
                    cv2.circle(frame, (x, y), 3, (0, 255, 0), -1) # Draw a green circle at each corner
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
        
        # Yield the frame in byte format
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

if __name__ == "__main__":
    detect_aruco_from_webcam()