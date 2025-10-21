
import subprocess
import os
import cv2
import numpy as np
import pdb
#from sklearn.neighbors import NearestNeighbors
import math
from visual import *
#from simplify_points import *
from skimage.morphology import medial_axis, skeletonize

# Setup directories
input_folder = "../data"
output_folder = "../output"
src_file = "alphabets.png"
output = "gcode.out"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# user-configurable drawing dimension in pixels
INPUT_WIDTH = 500 
INPUT_HEIGHT = 500

 

class GcodeConverter:
    def __init__(self):
        self.M_UP = "M5"     # marker up
        self.M_DOWN = "M3"   # marker down
        self.POSITION = "G0" # robot positioning to starting point (fastest speed)
        self.TRACE = "G1"    # robot tracing a path to next point (slower speed)
   
    
    def preprocess(self, image):
        img = cv2.imread(image)
        if img is None:
            raise ValueError(f"Could not load image from {image}")
    
        # Resize image
        img = cv2.resize(img, (INPUT_WIDTH, INPUT_HEIGHT))
    

        # TO_DO: use a mask for processing 3 different color threshold 
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
 

        # Use smaller kernel, fewer iterations
        kernel = np.ones((3, 3), np.uint8)  # Much smaller
        binary = cv2.dilate(binary, kernel, iterations=1)  # Just onc

        # Skeletonize to get center lines
        binary_bool = binary > 0
        skeleton, _ = medial_axis(binary_bool, return_distance=True)
        cleaned = (skeleton * 255).astype(np.uint8)



        return img, cleaned
    


    def filter_duplicates(self, contours, min_distance=1):
        """
        Remove contours whose bounding boxes are too close
        """
        if len(contours) <= 1:
            return contours
        

        bounding_boxes = [cv2.boundingRect(cnt) for cnt in contours]
        
        unique_contours = []
        used_indices = set()
        
        for i, (x1, y1, w1, h1) in enumerate(bounding_boxes):
            if i in used_indices:
                continue
                
            unique_contours.append(contours[i])
            used_indices.add(i)
            
            for j, (x2, y2, w2, h2) in enumerate(bounding_boxes):
                if j not in used_indices:
                    # Calculate center distance
                    cx1, cy1 = x1 + w1/2, y1 + h1/2
                    cx2, cy2 = x2 + w2/2, y2 + h2/2
                    distance = np.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2)
                    
                    if distance < min_distance:
                        used_indices.add(j)
                        print(f"Removed nearby contour {j} (distance: {distance:.1f})")
        
        return unique_contours
    
  
    """sort contours from top to bottom, left to right"""
    def sort_contours(self, contours, row_height_threshold=50):
        # Get the height and width for each contour 
        bounding_boxes = [cv2.boundingRect(cnt) for cnt in contours]
        
        # sorts the contours based on increasing order of Y coordinates (top to bottom)
        contours_with_boxes = list(zip(contours, bounding_boxes))
        contours_with_boxes.sort(key=lambda x: x[1][1])  
        
        # Group contours into rows
        rows = []
        current_row = []
        current_y = contours_with_boxes[0][1][1] if contours_with_boxes else 0
        
        for cnt, (x, y, w, h) in contours_with_boxes:
            if abs(y - current_y) > row_height_threshold:
                # New row if new contour's y coordinate is higher than current row
                if current_row:
                    rows.append(current_row)
                current_row = [(cnt, (x, y, w, h))]
                current_y = y
            else:
                # Same row 
                current_row.append((cnt, (x, y, w, h)))
        
        if current_row:
            rows.append(current_row)
        
        sorted_contours = []
        sorted_boxes = []
        
        # Sort by increasing order of X within the same row
        for row in rows:
            row.sort(key=lambda x: x[1][0])  
            for cnt, box in row:
                sorted_contours.append(cnt)
                sorted_boxes.append(box)
    
        return sorted_contours, sorted_boxes


    """Write gcode to output file"""
    def image_to_gcode(self, image_path, output_file, threshold_Closed = 0.3, precision=0.005):
        _, binary_img = self.preprocess(image_path)

        # Find contours and sort it
        contours_org, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = self.filter_duplicates(contours_org)

        contours = [c for c in contours if len(c) > 5]  # filter ouxt small contours
        contours, _ = self.sort_contours(contours)

        # Open file for writing
        with open(output_file, 'w') as f:
            f.write("G90\n")  # Move to origin (0,0) at top left corner 

            print(f"Found {len(contours)} contours")

            for i, contour in enumerate(contours):
                # check if the contour is closed or open path. 
                all_points = contour.reshape(-1, 2) 
                points = all_points
                start = all_points[0]
                end = all_points[-1]
                distance = np.linalg.norm(start - end)

                # compare the distance to contour size
                x, y, w, h = cv2.boundingRect(contour)
                contour_size = np.linalg.norm([w, h])
                threshold = contour_size * threshold_Closed
                isClosed = False
                if distance < threshold:  
                    isClosed = True     # if distance between start and end is close, close the path
            
                # simplify the points in the contour 
                perimeter = cv2.arcLength(contour, True)
                epsilon = precision * perimeter   #(adjustable for shape precision)
                simplified = cv2.approxPolyDP(contour, epsilon, True)
                points = simplified.reshape(-1, 2)

                if len(points) < 2:
                    continue 

                f.write(f"Contour {i+1}\n")
                
                # Move to first point (marker up)
                f.write(f"G0 X{points[0][0]:.2f} Y{points[0][1]:.2f}\n")
                
                # Marker down, draw the contour
                f.write(f"{self.M_DOWN}\n")
                for point in points[1:]:
                    f.write(f"G1 X{point[0]:.2f} Y{point[1]:.2f}\n")

                if isClosed:      # connect end to start if closed path
                    f.write(f"G1 X{points[0][0]:.2f} Y{points[0][1]:.2f}\n")

                f.write(f"{self.M_UP}\n")
                f.write("\n")

            f.write("M30\n")      # end of drawing, program completes

        print(f"G-code successfully written to {output_file}")
        return output_file
    


def main():
    converter = GcodeConverter()

    try:
 
        image_path = f"{input_folder}/{src_file}" 
        output_file = converter.image_to_gcode(image_path, f"{output_folder}/{output}")
        print(f"G-code saved to: {output_file}")
        visualize_g1_coordinates(output_file)

    except Exception as e:
        print(f"Error with custom image: {e}")


if __name__ == "__main__":
    main()
    
