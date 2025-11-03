

import subprocess
import os
import cv2
import numpy as np
import pdb
import math
from visual import *
#from skimage.morphology import medial_axis, skeletonize
import pytesseract    # for text identification (to be implemented)
from scipy.spatial import distance



# Setup directories
input_folder = "../color_img"
output_folder = "../color_output"
src_file = "color_shapes.png"
gcode_file = src_file.rsplit('.', 1)[0] + ".gcode"
gcode_plotfile = src_file.rsplit('.', 1)[0] + ".png"


if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# user-configurable drawing dimension in pixels 
INPUT_WIDTH = 1500
INPUT_HEIGHT = 1500

 

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

        # determine which filter to use based on image quality, Guassian for poorer quality. filter can make image worse
        #blurred = cv2.GaussianBlur(gray, (5,5), 0)
        #blurred = cv2.bilateralFilter(gray, 9, 75, 75)

        # Preferably use threshold. adaptiveThreshold is for photos with poor lighting
        #_, binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2 )
        _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
 

        # Connect small gaps to get smoother shapes
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        # smooth thinning but removes minor details
        skeleton = cv2.ximgproc.thinning(cleaned.astype(np.uint8), thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

        return img, skeleton


    def sort_contours(self, contours, row_height_threshold=50):
        """sort contours from top to bottom, left to right"""

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


 

    def remove_duplicates(self, contours, hierarchy):
        """Removes obvious parent-child duplicates"""

        if hierarchy is None:
            return contours
        
        
        hierarchy = hierarchy[0]
        keep_indices = []
        
        for i in range(len(contours)):
            parent_idx = hierarchy[i][3]
            
            # Keep all parent contours
            if parent_idx == -1:
                keep_indices.append(i)
                continue
            
            # For child contours, check if they're duplicates
            parent_contour = contours[parent_idx]
            child_contour = contours[i]
            
            # bbox-based duplicate check
            if not self.is_bbox_duplicate(parent_contour, child_contour):
                keep_indices.append(i)
        
        return [contours[i] for i in keep_indices]


    def remove_contour_duplicates(self, contours):
        processed = set()
        keep_indices = []
    
        for i in range(len(contours)):
            if i in processed:
                continue
                
            keep_indices.append(i)
            current_contour = contours[i]
            
            # Compare with all other contours   
            for j in range(i + 1, len(contours)):
                if j in processed:
                    continue
                    
                if self.is_bbox_duplicate(current_contour, contours[j]):
                    processed.add(j)
        
        return [contours[i] for i in keep_indices]


    def is_bbox_duplicate(self, parent, child):
        """Check for parent/child duplicates in the hierarchy by comparing their bounding-box"""

        px, py, pw, ph = cv2.boundingRect(parent)
        cx, cy, cw, ch = cv2.boundingRect(child)
        
        # Calculate overlap and size ratios
        overlap_x = max(0, min(px + pw, cx + cw) - max(px, cx))
        overlap_y = max(0, min(py + ph, cy + ch) - max(py, cy))
        overlap_area = overlap_x * overlap_y
        
        parent_area = pw * ph
        child_area = cw * ch
        
        # If child mostly overlaps parent and has similar size, it's a duplicate
        overlap_ratio = overlap_area / child_area if child_area > 0 else 0
        size_ratio = child_area / parent_area if parent_area > 0 else 0
        
        return overlap_ratio > 0.8 and size_ratio > 0.6


    # adjustable min_distance threshold
    def remove_close_points(self, points, min_distance=2.0): 
        """Remove close points after approxPolyDP"""

        if len(points) < 3:
            return points
        
        filtered_points = [points[0]]  # Always keep first point
        
        for i in range(1, len(points)):
            current_point = points[i]
            last_kept_point = filtered_points[-1]
            
            distance = np.sqrt(np.sum((current_point - last_kept_point) ** 2))
            
            if distance >= min_distance:
                filtered_points.append(current_point)
            # else: skip this point (too close)

        return np.array(filtered_points)



    def image_to_gcode(self, image_path, output_file, threshold_Closed = 0.3, precision=0.007):
        """Write gcode to output file"""
        _, binary_img = self.preprocess(image_path)

        # Find contours using RETR_TREE (detects nested contours)
        contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

         # filter out small contours
        contours = [c for c in contours if len(c) > 3] 

        # remove inner regions that are almost identical to the parent in the hierarchy
        contours = self.remove_contour_duplicates(contours)

        # sort the contours from left to right, top to bottom
        contours, _ = self.sort_contours(contours)

      
        # Open file for writing
        with open(output_file, 'w') as f:
            f.write("G90\n")  # Move to origin (0,0) at top left corner 

            print(f"Found {len(contours)} contours")

            for i, contour in enumerate(contours):
                # check if the contour is closed or open path. 
                points = contour.reshape(-1, 2) 
                start = points[0]
                end = points[-1]
                distance = np.linalg.norm(start - end)

                # if distance between start and end is close, close the path
                x, y, w, h = cv2.boundingRect(contour)
                contour_size = np.linalg.norm([w, h])
                threshold = contour_size * threshold_Closed
                isClosed = False
                if distance < threshold:  
                    isClosed = True     
            
                # simplify the points in the contour 
                perimeter = cv2.arcLength(contour, True)
                epsilon = precision * perimeter         # (adjustable for shape precision)
                simplified = cv2.approxPolyDP(contour, epsilon, True)
                points = simplified.reshape(-1, 2)

                if len(points) < 2:
                    continue 

                points = self.remove_close_points(points)   # remove close points after simplification

                f.write(f"Contour {i+1}\n")
                
                # Move to first point (marker up)
                f.write(f"G0 X{points[0][0]} Y{points[0][1]}\n")
                
                # Marker down, draw the contour
                f.write(f"{self.M_DOWN}\n")
                for point in points[1:]:
                    f.write(f"G1 X{point[0]} Y{point[1]}\n")

                if isClosed:      # connect end to start if closed path
                    f.write(f"G1 X{points[0][0]} Y{points[0][1]}\n")

                f.write(f"{self.M_UP}\n")
                f.write("\n")

            f.write("M30\n")      # end of drawing, program completes

        print(f"G-code successfully written to {output_file}")
        return output_file
    


def main():
    converter = GcodeConverter()

    try:
 
        image_path = f"{input_folder}/{src_file}" 
        output_file = converter.image_to_gcode(image_path, f"{output_folder}/{gcode_file}")
        print(f"G-code saved to: {output_file}")
        visualize_gcode(output_file, gcode_plotfile)

    except Exception as e:
        print(f"Error with custom image: {e}")


if __name__ == "__main__":
    main()
    
