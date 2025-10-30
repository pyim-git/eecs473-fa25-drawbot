
import subprocess
import os
import cv2
import numpy as np
import pdb
import math
from visual import *
from color import *
import pytesseract    # for text identification (to be implemented)
from scipy.spatial import distance



# Setup directories
input_folder = "../data"
output_folder = "../output"
src_file = "circle.png"
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
   
    
    def preprocess(self, image_path):
        """resize, clean, and skeletonize the image"""
        org_img = cv2.imread(image_path)
        if org_img is None:
            raise ValueError(f"Could not load image from {image_path}")

        org_img = cv2.resize(org_img, (INPUT_WIDTH, INPUT_HEIGHT))

        # convert image to gray scale 
        gray = cv2.cvtColor(org_img, cv2.COLOR_BGR2GRAY)

        # determine which filter to use based on image quality, Guassian for poorer quality. filter can make image worse
        #blurred = cv2.GaussianBlur(gray, (5,5), 0)
        #blurred = cv2.bilateralFilter(gray, 9, 75, 75)

        # Preferably use threshold. adaptiveThreshold is for photos with poor lighting
        #_, binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2 )
        _, binary = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY_INV)

        # # Connect small gaps to get smoother shapes
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close, iterations=1)

        # for clustered text, separate them using this function
        # kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel_open, iterations=1)

        # smooth thinning but removes minor details
        skeleton = cv2.ximgproc.thinning(cleaned.astype(np.uint8), thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        return skeleton, org_img


    def sort_contours(self, contours, row_height_threshold=100):
        """sort contours from top to bottom, left to right"""

        # Get the height and width for each contour 
        bounding_boxes = [cv2.boundingRect(item['contour']) for item in contours]
        
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



    def sort_split_contours(self, contours, split_y=100):
        """sort contours by row number (based on split_y), then left to right"""
        
        contours_with_boxes = []
        for contour_data in contours:
            contour = contour_data['contour']
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate which row this contour belongs to
            row_number = y // split_y
            
            contours_with_boxes.append({
                'contour_data': contour_data,
                'bbox': (x, y, w, h),
                'row_number': row_number
            })
        
        # Sort by row number first, then by X coordinate within each row
        contours_with_boxes.sort(key=lambda item: (item['row_number'], item['bbox'][0]))
        
        # Extract sorted contours
        sorted_contours = [item['contour_data'] for item in contours_with_boxes]
        sorted_boxes = [item['bbox'] for item in contours_with_boxes]
        
        return sorted_contours, sorted_boxes


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



    def is_bbox_duplicate(self, contourA, contourB):
        """Check for contour duplicates by comparing their bounding boxes"""

        px, py, pw, ph = cv2.boundingRect(contourA)
        cx, cy, cw, ch = cv2.boundingRect(contourB)
        
        # Calculate overlap and size ratios
        overlap_A = max(0, min(px + pw, cx + cw) - max(px, cx))
        overlap_B = max(0, min(py + ph, cy + ch) - max(py, cy))
        overlap_area = overlap_A * overlap_B
        
        areaA= pw * ph
        areaB = cw * ch
        
        # If child mostly overlaps parent and has similar size, it's a duplicate
        overlap_ratio = overlap_area / areaB if areaB > 0 else 0
        size_ratio = areaB / areaA if areaA > 0 else 0
        
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
    

    def simplify_points(self, contour, threshold_Closed = 0.3, precision=0.007 ):
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
                
        # remove close points after simplification
        points = self.remove_close_points(points)   

        if isClosed: # connect end to start if closed path
            points = np.vstack([points, points[0]])

        return points
    
     

    def image_to_gcode(self, image_path, output_file):
        """wwrite gcode to output file"""

        # process the image to get clean contours
        processed_img, org_img = self.preprocess(image_path)

        # Find contours using RETR_TREE (detects nested contours)
        contours, _ = cv2.findContours(processed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # filter out small contours
        contours = [c for c in contours if len(c) > 3] 

        # remove inner regions that are almost identical to the parent in the hierarchy
        contours = self.remove_contour_duplicates(contours)

        # assign colors to all contours
        color_contours = assignColors(org_img, contours)

        split_contours = []
        # simplify the contours by pruning away points
        for i, item in enumerate(color_contours):
            contour = item['contour']
            color = item['color']
            points = self.simplify_points(contour).astype(np.float32)

            # split contour into horizontal sections
            contour = np.array(points, dtype=np.float32).reshape(-1, 1, 2)

            split_contours.extend(splitContoursHorizontally(contour, color))

        # sort the contours from left to right, top to bottom
        sorted_contours, b_boxes = self.sort_split_contours(split_contours)


        # Open file for writing
        with open(output_file, 'w') as f:
            f.write("G90\n")  # Move to origin (0,0) at top left corner 

            print(f"Found {len(sorted_contours)} contours")

            for i, item in enumerate(sorted_contours):
                color = item['color']
                contour = item['contour']

            
               # for item in split_contour:
                    # contour = item['contour']
                points = contour.reshape(-1, 2)
                if len(points) ==0:
                    print("empty points")
                    continue

                #points = 5*points  # scale to drawing dimensions mm?

                    
                f.write(f"Contour {i+1}\n")
                f.write(f"Color {color}\n")

                # Move to first point (marker up)
                f.write(f"G0 X{points[0][0]:.5f} Y{points[0][1]:.5f}\n")
                
                # Marker down, draw the contour
                f.write(f"{self.M_DOWN}\n")
                for point in points[1:]:
                    f.write(f"G1 X{point[0]:.5f} Y{point[1]:.5f}\n")

                # if isClosed:      # connect end to start if closed path
                #     f.write(f"G1 X{points[0][0]} Y{points[0][1]}\n")
            
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
        visualize_gcode(output_file, f"{output_folder}", gcode_plotfile)

    except Exception as e:
        print(f"Error with custom image: {e}")


if __name__ == "__main__":
    main()
    