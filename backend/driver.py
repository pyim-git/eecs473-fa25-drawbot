
import subprocess
import os
import cv2
import numpy as np
import pdb
import math
from visual import *
from color import *
from ocr import *


# Setup directories
input_folder = "backend/uploads"
output_folder = "backend/color_output"
src_file = "myimage.png"
gcode_file1 = src_file.rsplit('.', 1)[0] + "1.gcode"
gcode_file2 = src_file.rsplit('.', 1)[0] + "2.gcode"
gcode_plotfile = src_file.rsplit('.', 1)[0] + ".png"


if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# default pixel dimensions set for clean image processing
WIDTH_PIXELS = 2000
HEIGHT_PIXELS = 2000
RowHeight = 100 # in mm

#3 pixels = 1 mm

class GcodeConverter:
    def __init__(self, width_mm, height_mm, image_type):
        self.M_UP = "M5"     # marker up
        self.M_DOWN = "M3"   # marker down
        self.POSITION = "G0" # robot positioning to starting point (fastest speed)
        self.TRACE = "G1"    # robot tracing a path to next point (slower speed)
        self.isDigital = (image_type.lower() == 'digital')
        self.WIDTH_MM = width_mm
        self.HEIGHT_MM = height_mm
   
    
    def preprocess_photo(self, image_path):
        """resize, clean, and skeletonize the image"""
        org_img = cv2.imread(image_path)
        if org_img is None:
            raise ValueError(f"Could not load image from {image_path}")

        org_img = cv2.resize(org_img, (self.WIDTH_MM, self.HEIGHT_MM))
        hsv_img = cv2.cvtColor(org_img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)


        # create color masks: filters out all gray shadow regions
        hue, saturation, value = cv2.split(hsv_img)
        _, nongray_mask = cv2.threshold(saturation, 35, 255, cv2.THRESH_BINARY) 
        _, black_mask = cv2.threshold(value, 60, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.bitwise_or(nongray_mask,black_mask)

       
        # _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # turns white and gray colors to background color
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
        filtered_binary = cv2.bitwise_and(binary, binary, mask=mask)
     
        # connect shape by closing gaps between pixels
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2))
        cleaned = cv2.morphologyEx(filtered_binary, cv2.MORPH_OPEN, kernel, iterations=1)

        # thinning: detects center line and convert it into a single path
        skeleton = cv2.ximgproc.thinning(cleaned.astype(np.uint8), 
                                    thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        return skeleton, cleaned, org_img

   
    def preprocess_digital(self, image_path):
        """resize, clean, and skeletonize the image"""
        org_img = cv2.imread(image_path)
        if org_img is None:
            raise ValueError(f"Could not load image from {image_path}")

        org_img = cv2.resize(org_img, (self.WIDTH_MM, self.HEIGHT_MM))

        # convert image to binary
        gray = cv2.cvtColor(org_img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 205, 255, cv2.THRESH_BINARY_INV)
        #_, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        # # Connect small gaps to get smoother shapes
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        # cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        

        skeleton = cv2.ximgproc.thinning(cleaned.astype(np.uint8), thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        #cv2.imshow('skeleton', skeleton)

        return skeleton, cleaned, org_img


    def findShapes(self, binary, skeleton_contours, result, thresholdClosed = 0.3, shape_similarity_threshold=0.15, size_diff = 0.03):
        """fix the skeletonized shapes with perfect shapes"""

        # Detect unskeletonized contours using RETR_TREE to detect nested contours
        contours, hierarchy= cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        hierarchy = hierarchy[0]
        skeletons = list(skeleton_contours)

        if (len(hierarchy) == 0):
            return skeletons
        
        # loop through all outer contours
        for i, outer_contour in enumerate(contours):

            perimeter = cv2.arcLength(outer_contour, True)
            epsilon = 0.03 * perimeter        
            simplified = cv2.approxPolyDP(outer_contour, epsilon, True)
            simplified = simplified.reshape(-1, 2)

            if (cv2.contourArea(simplified) < 100):
                continue

            # only detect closed shapes
            points = outer_contour.reshape(-1, 2) 
            start = points[0]
            end = points[-1]
            distance = np.linalg.norm(start - end)

            # check the distance between start and end to determine if closed shape
            x, y, w, h = cv2.boundingRect(outer_contour)
            contour_size = np.linalg.norm([w, h])
            threshold = contour_size * thresholdClosed
            if distance >= threshold:  
                continue


            if hierarchy[i][3] == -1:  # Outer contour 
                first_child_idx = hierarchy[i][2]
                
                # loop through all child contours and look for similar shape to parent
                inner_contour = contours[first_child_idx]
                if (cv2.contourArea(inner_contour) < 50):
                    continue
                
                # Check shape similarity
                score = cv2.matchShapes(outer_contour, inner_contour, cv2.CONTOURS_MATCH_I1, 0)
                if score < shape_similarity_threshold:

                    # check that inner and outer contours have close proximity
                    x1, y1, w1, h1 = cv2.boundingRect(outer_contour)
                    x2, y2, w2, h2 = cv2.boundingRect(inner_contour)
                    avg_size = (max(w1, h1) + max(w2, h2)) / 2
                    
                    # Distance threshold = percentage of average size
                    adaptive_threshold = avg_size * size_diff
                    
                    # Check centroid distance
                    centroid1 = np.mean(outer_contour.reshape(-1, 2), axis=0)
                    centroid2 = np.mean(inner_contour.reshape(-1, 2), axis=0)
                    pixel_distance = np.linalg.norm(centroid1 - centroid2)

                    if (pixel_distance < adaptive_threshold):
                        self.replaceShapes(result,outer_contour, inner_contour, skeletons)       
                                 
        # cv2.drawContours(result, contours, -1, (0, 75, 150), 2) # Brown

        return skeletons


    def replaceShapes(self, result, outerContour, innerContour, skeletons, thresholdClosed = 0.3):
        # loop through all the skeleton contours to find one enclosed between inner and outer contours
        outer_x,outer_y, outer_w, outer_h = cv2.boundingRect(outerContour)
        inner_x, inner_y, inner_w, inner_h = cv2.boundingRect(innerContour)

        outerArea = cv2.contourArea(outerContour)
        innerArea = cv2.contourArea(innerContour)

        for i, skeleton in enumerate(skeletons):
            # find the skeleton thats between inner and outer contour
            x,y,w,h = cv2.boundingRect(skeleton)

            if not (outer_x < x < inner_x):
                continue
            
            if not (outer_y < y < inner_y):
                continue

            skeletonArea = cv2.contourArea(skeleton)
            if not (innerArea < skeletonArea < outerArea ):
                continue

            # only replace closed shapes
            points = skeleton.reshape(-1, 2) 
            start = points[0]
            end = points[-1]
            distance = np.linalg.norm(start - end)

            # check the distance between start and end to determine if closed shape
            x, y, w, h = cv2.boundingRect(skeleton)
            contour_size = np.linalg.norm([w, h])
            threshold = contour_size * thresholdClosed
            if distance < threshold:  
                skeletons[i] = outerContour
                break

        return skeletons

    def sort_split_contours(self, contours, row_height):
        """sort contours by rows
            row height = gantry length"""
        
        contours_with_boxes = []
        for contour_data in contours:
            contour = contour_data['contour']
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate which row this contour belongs to
            row_number = y // row_height

            # sort points left to right in even rows
            if (row_number % 2 ==0):
                if (contour[0][0][0] > contour[-1][0][0]):
                    contour_data['contour'] = contour[::-1]
            else: # sort points right to left in odd rows
                if (contour[0][0][0] < contour[-1][0][0]):
                    contour_data['contour'] = contour[::-1]
            
            contours_with_boxes.append({
                'contour_data': contour_data,
                'bbox': (x, y, w, h),
                'row_number': row_number
            })
        
        # Sort by row number first, then X left->right alternate with X right->left
        contours_with_boxes.sort(key=lambda item: (item['row_number'], (item['bbox'][0] + item['bbox'][2]) if 
                                                  (item['row_number']%2 == 0) else -item['bbox'][0]-item['bbox'][2]))
        
        # Extract sorted contours
        sorted_contours = [item['contour_data'] for item in contours_with_boxes]
        
        return sorted_contours


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



    def is_bbox_duplicate(self, contourA, contourB, shape_threshold = 0.3, close_threshold= 3):
        """Check for contour duplicates by comparing their bounding boxes"""
 
        px, py, pw, ph = cv2.boundingRect(contourA)
        cx, cy, cw, ch = cv2.boundingRect(contourB)

        # Check centroid distance
        centroidA = np.mean(contourA.reshape(-1, 2), axis=0)
        centroidB = np.mean(contourB.reshape(-1, 2), axis=0)
        pixel_distance = np.linalg.norm(centroidA - centroidB)
        
        # Calculate overlap and size ratios
        overlap_A = max(0, min(px + pw, cx + cw) - max(px, cx))
        overlap_B = max(0, min(py + ph, cy + ch) - max(py, cy))
        overlap_area = overlap_A * overlap_B
        
        areaA= pw * ph
        areaB = cw * ch
        
        # If child mostly overlaps parent and has similar size, it's a duplicate
        overlap_ratio = overlap_area / areaB if areaB > 0 else 0
        size_ratio = areaB / areaA if areaA > 0 else 0
        
        return overlap_ratio > 0.8 and size_ratio > 0.7 and pixel_distance < close_threshold


    # adjustable min_distance threshold
    def remove_close_points(self, points, min_distance): 
        """Remove close points to increase step size"""
        if len(points) < 2:
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
    

    def simplify_points(self, contour, threshold_Closed = 0.3):
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
        if perimeter > 4000:
            simplify = 0.001
        if perimeter > 2000:
            simplify = 0.002
        elif perimeter > 900:
            simplify = 0.003
        elif perimeter > 250:
            simplify = 0.0035
        elif perimeter > 80:
            simplify = 0.004
        elif perimeter > 30:
            simplify = 0.05
        else:
            simplify = 0.07


        epsilon = (simplify) * perimeter    
        simplified = cv2.approxPolyDP(contour, epsilon, True)
        points = simplified.reshape(-1, 2).astype(np.float32)

        if isClosed:    # connect end to start if closed path
            points = np.vstack([points, points[0]])

        # contour simplification: prune away points in line segments
        length = cv2.arcLength(points, closed=True)
        stepsize = 1
        if (length > 500):
            stepsize = 9
        elif (length > 300):
            stepsize = 7
        elif (length > 200):
            stepsize = 5
        elif (length > 100):
            stepsize = 4
        elif (length > 50):
            stepsize = 3
        elif (length > 20):
            stepsize = 2

        # remove nearby points
        simplified_points = self.remove_close_points(points, stepsize)   

        # scale the pixels to millimeters 
        scale_x = self.WIDTH_MM / WIDTH_PIXELS
        scale_y = self.HEIGHT_MM / HEIGHT_PIXELS
        scaled_points = simplified_points
        scaled_points[:, 0] *= scale_x
        scaled_points[:, 1] *= scale_y
        points = scaled_points

        return points
    
    def filters_contours(self, skeleton_contours, skeleton_img):

        """filters out small outlier paths"""

        filtered_contours = []



        for i, contour in enumerate(skeleton_contours):

            if (cv2.arcLength(contour, True) > 25):

                filtered_contours.append(contour)

            else:



                x, y, w, h = cv2.boundingRect(contour)

                roi = skeleton_img[y:y+w, x:x+h]

                contour_pixels = cv2.countNonZero(roi)



                region_y = max(y+w+12, HEIGHT_PIXELS)

                region_x = max(x+h+12, WIDTH_PIXELS)

                roi = skeleton_img[y:region_y, x:region_x]

                region_pixels = cv2.countNonZero(roi)



                if (region_pixels >= contour_pixels):

                    filtered_contours.append(contour)



        return filtered_contours
   
    def image_to_gcode(self, image_path, output_file1, output_file2):
        """write gcode to output file"""
        # process the image to get clean contours
        if self.isDigital:
            print("Running OCR for digital image")
            detected_words = detect_text(image_path, self.WIDTH_MM, self.HEIGHT_MM)
            image_text_path = transpose_print_text_to_image(detected_words, image_path)
            skeleton_img, binary, org_img = self.preprocess_digital(image_text_path)
            org_img = cv2.imread(image_path)
            org_img = cv2.resize(org_img, (self.WIDTH_MM, self.HEIGHT_MM))
        else:
            print("Processing photo image")
            skeleton_img, binary, org_img = self.preprocess_photo(image_path)
        # Find contours using RETR_TREE (detects nested contours)
        skeleton_contours, _ = cv2.findContours(skeleton_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        height, width = org_img.shape[:2]
        result = np.zeros((height, width, 3), dtype=np.uint8)
    
        # detect triangles and diamonds, and make them look nicer
        skeleton_contours = self.findShapes(binary, skeleton_contours, result)

        # filter out small contours
        contours = self.filters_contours(skeleton_contours, skeleton_img)

        # remove similar contours
        contours = self.remove_contour_duplicates(contours)

        # assign colors to all contours
        color_contours = assignColors(org_img, contours, self.isDigital)

        # TO DIVIDE CONTOURS INTO ROWS

        split_contours = []

        for i, item in enumerate(color_contours):
            contour = item['contour']
            color = item['color']
            # simplify points in path and rescale points to mm
            points = self.simplify_points(contour).astype(np.float32)
            # split contour into horizontal sections
            contour = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
            split_contours.extend(splitContoursHorizontally(contour, color, RowHeight))
        # sort the contours from left to right, top to bottom

        sorted_contours = self.sort_split_contours(split_contours, RowHeight)
        # Open file for writing: file1 for absolute positions, file2 for relative offsets

        print("contours done")
        # Open file for writing
        with open(output_file1, 'w') as f1, open(output_file2, 'w') as f2:
            print(f"Found {len(sorted_contours)} contours")

            for i, item in enumerate(sorted_contours):

                color = item['color']

                contour = item['contour']

                points = contour.reshape(-1, 2)

                # Contour heading
                f1.write(f"Contour {i+1}\n")
                f1.write(f"Color {color}\n")

                f2.write(f"Contour {i+1}\n")
                f2.write(f"Color {color}\n")

                # Move to first point (marker up)
                f1.write(f"G0 X{points[0][0]:.2f} Y{points[0][1]:.2f}\n")

                # get the angle and relative offset to first point from origin
                dist = math.sqrt(points[0][0]**2 + points[0][1]**2)
                angle_rad = math.atan2(points[0][1],points[0][0])

                f2.write(f"A {angle_rad:.3f}\n")
                f2.write(f"G0 {dist:.2f}\n")

                prev_point = points[0]

                # Marker down, draw the contour
                for point in points[1:]:
                    f1.write(f"G1 X{point[0]:.2f} Y{point[1]:.2f}\n")

                    # get the angle and relative offset between last and next point
                    dx = point[0] - prev_point[0]
                    dy = point[1] - prev_point[1]
                    angle_rad = math.atan2(dy, dx)
                    dist = math.sqrt(dx**2 + dy**2)  

                    f2.write(f"A {angle_rad:.3f}\n")
                    f2.write(f"G1 {dist:.2f}\n")

                    prev_point = point
                f2.write("\n")
                f1.write("\n")

            f1.write("M30\n")      # end of drawing, program completes
            f2.write("M30\n")      # end of drawing, program completes


        print(f"G-code successfully written to {output_file1} and {output_file2}")
        f1.close()
        f2.close()
        return output_file1, output_file2
    


def main():
    converter = GcodeConverter(1500, 1500, 'digital')
    try:
        image_path = f"{input_folder}/{src_file}" 
        output_file1, output_file2 = converter.image_to_gcode(image_path, f"{output_folder}/{gcode_file1}", f"{output_folder}/{gcode_file2}" )
        print(f"G-code saved to: {output_file1} and {output_file2}")
        visualize_gcode(output_file1, f"{output_folder}", gcode_plotfile)

    except Exception as e:
        print(f"Error with custom image: {e}")


if __name__ == "__main__":
    main()
    