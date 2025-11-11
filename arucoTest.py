import cv2
import cv2.aruco as aruco

def detect_aruco_from_webcam():
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

if __name__ == "__main__":
    detect_aruco_from_webcam()
