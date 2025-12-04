
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


# default pixel dimensions set for clean image processing
WIDTH_PIXELS = 2000
HEIGHT_PIXELS = 2000
RowHeight = 100 # in mm
# user configurable color selects
black_select = 'black'
blue_select = 'blue'
red_select = 'red'
green_select = 'green'
purple_select = 'purple'
orange_select = 'orange'
brown_select = 'brown'
yellow_select = 'yellow'
#3 pixels = 1 mm


class GcodeConverter:
    def __init__(self, width_mm, height_mm, image_type, color_map, top_left_x=0, top_left_y=0):
        self.POSITION = "G0" # robot positioning to starting point (fastest speed)
        self.TRACE = "G1"    # robot tracing a path to next point (slower speed)

        self.RowHeight = 100    # in mm
        self.WIDTH_PIXELS = 2000
        self.HEIGHT_PIXELS = 2000   

        # user configurable parameters
        self.DEFAULT_COLOR = 'black' # default color for undefined colors
        self.isDigital = (image_type.lower() == 'digital')
        self.WIDTH_MM = width_mm
        self.HEIGHT_MM = height_mm
        self.top_left_x = top_left_x
        self.top_left_y = top_left_y
        # user configurable default color for undetected colors
        self.digital_colors = color_map
        self.photo_colors = color_map
        self.digital_colors['gray'] = color_map['black']
         # user configurable robot drawing colors with corresponding marker positions
        color_set = set()
        for dig_color in self.digital_colors:
            color_set.add(self.digital_colors[dig_color])
        color_set = list(color_set)
        color1 = color_set[0]
        color2 = color_set[1]
        color3 = color_set[2]
        self.draw_positions = {
            color1: 1,
            color2: 2,
            color3: 3
        }
    
    def preprocess_photo(self, image_path):
        """resize, clean, and skeletonize the image"""
        org_img = cv2.imread(image_path)
        if org_img is None:
            raise ValueError(f"Could not load image from {image_path}")

        org_img = cv2.resize(org_img, (self.WIDTH_PIXELS, self.HEIGHT_PIXELS))
        hsv_img = cv2.cvtColor(org_img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)


        # create color masks: filters out all gray shadow regions
        hue, saturation, value = cv2.split(hsv_img)
        _, nongray_mask = cv2.threshold(saturation, 35, 255, cv2.THRESH_BINARY) 
        _, black_mask = cv2.threshold(value, 60, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.bitwise_or(nongray_mask,black_mask)

       
        # turns white and gray colors to background color
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
        filtered_binary = cv2.bitwise_and(binary, binary, mask=mask)
     
        # for clustered text, this function creates whitespace between pixels
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

        org_img = cv2.resize(org_img, (self.WIDTH_PIXELS, self.HEIGHT_PIXELS))

        # convert image to binary
        gray = cv2.cvtColor(org_img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)

        # create gaps between pixels to get whitespace for images with heavy text
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        

        skeleton = cv2.ximgproc.thinning(cleaned.astype(np.uint8), thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        #cv2.imshow('skeleton', skeleton)

        return skeleton, cleaned, org_img


    def findShapes(self, binary, skeleton_contours, shape_similarity_threshold=0.15, size_diff = 0.03):
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
                    
                    adaptive_threshold = avg_size * size_diff
                    
                    # Check centroid distance
                    centroid1 = np.mean(outer_contour.reshape(-1, 2), axis=0)
                    centroid2 = np.mean(inner_contour.reshape(-1, 2), axis=0)
                    pixel_distance = np.linalg.norm(centroid1 - centroid2)

                    if (pixel_distance < adaptive_threshold):
                        self.replaceShapes(outer_contour, inner_contour, skeletons)       
                                 
        # cv2.drawContours(result, contours, -1, (0, 75, 150), 2) # Brown

        return skeletons


    def replaceShapes(self, outerContour, innerContour, skeletons):
       # loop through all the skeleton contours to find one enclosed between inner and outer contour
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
            if not (innerArea < skeletonArea < outerArea):
                continue

            skeletons[i] = outerContour

        return skeletons

    def sort_split_contours(self, contours):
        """sort contours by rows
            row height = gantry length"""
        
        # store list of contours for each row
        contours_data = [[] for _ in range(math.ceil(self.HEIGHT_MM/self.RowHeight))] 

        # stores the leftmost and rightmost x coordinate for each row
        row_bounds = [(None, None) for _ in range(math.ceil(self.HEIGHT_MM/self.RowHeight))]

        # get row information and bounds for each row
        for contour_data in contours:
            contour = contour_data['contour']
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate which row this contour belongs to
            row_num = y // self.RowHeight 
            left = min(contour[0][0][0], contour[-1][0][0])
            right = max(contour[0][0][0], contour[-1][0][0])
            contours_data[row_num].append({
                'contour_data': contour_data,
                'bounds': [left, right]
            })

            # update row leftmost and rightmost bounds
            if (row_bounds[row_num][0] is None):
                row_bounds[row_num] = (left, right)
            else:
                row_bounds[row_num] = (min(row_bounds[row_num][0], left), max(row_bounds[row_num][1], right))


        # sort all paths 
        prevRow_x = 0
        sorted_contours = []
        for row_num, row_contours in enumerate(contours_data):
            if (len(row_contours) > 0) :

                # determine row direction based on previous row's endpoint
                if abs(row_bounds[row_num][0] - prevRow_x) < abs(row_bounds[row_num][1] - prevRow_x):
                   # Left->Right (sort by contour's rigtmost endpoint)
                    row_contours.sort(key=lambda item: item['bounds'][1])    
                else:
                    # Right->Left
                    row_contours.sort(key=lambda item: -item['bounds'][0])

                merged_contours = self.merge_contours([item['contour_data'] for item in row_contours])
                row_contours = self.remove_path_retrace(merged_contours)
                
                # for each path, determine traversal direction: forward or reverse
                prev_x = prevRow_x
                for i, item in enumerate(row_contours):

                    contour = row_contours[i]['contour']
                    if (abs(contour[-1][0][0] - prev_x) <  abs(contour[0][0][0] - prev_x)):
                        # trace the path in reverse order if endpoint is closer to previous endpoint than path's start point 
                        row_contours[i]['contour'] = contour[::-1]
                        prev_x = contour[0][0][0]
                    else:
                        prev_x = contour[-1][0][0] # update endpoint

                prevRow_x = prev_x
                sorted_contours.extend(row_contours)
        return sorted_contours


    def merge_contours(self, contours):
        # merge contours if end1 = start2
        merged_contours = [] 
        current_chain = None
        last_color = None

        for item in contours:
            contour = item['contour']

            if current_chain is None:
                current_chain = contour
                last_color = item['color']
            else:
                # Check if current contour connects to the chain
                endpoint = current_chain[-1][0]
                startpoint = contour[0][0]
                dist = np.linalg.norm(endpoint - startpoint)
                if dist < 3 and item['color'] == last_color:
                    # if end1= start2, merge
                    current_chain = np.vstack([current_chain[:-1], contour])
                else:
                    merged_contours.append({'contour': current_chain, 'color': last_color})
                    current_chain = contour
                    last_color = item['color']
        
        if current_chain is not None:
            merged_contours.append({'contour': current_chain, 'color': last_color})
        
        return merged_contours
    

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



    def is_bbox_duplicate(self, contourA, contourB,  close_threshold= 3):
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
        """ prune away points in path
        threshold_Closed determines if a path is a closed path"""
        points = contour.reshape(-1, 2) 

        # check if the contour is closed or open path. 
        start = points[0]
        end = points[-1]
        distance = np.linalg.norm(start - end)
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

        # contour simplification: prune away points in line segments
        epsilon = (simplify) * perimeter    
        simplified = cv2.approxPolyDP(contour, epsilon, True)
        points = simplified.reshape(-1, 2).astype(np.float32)


        if isClosed:    # connect end to start if closed path
            points = np.vstack([points, points[0]])

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
    
        return simplified_points
    


    
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

                region_y = max(y+w+12, self.HEIGHT_PIXELS)
                region_x = max(x+h+12, self.WIDTH_PIXELS)
                roi = skeleton_img[y:region_y, x:region_x]
                region_pixels = cv2.countNonZero(roi)

                if (region_pixels >= contour_pixels):
                    filtered_contours.append(contour)

        return filtered_contours

        
    
    def splitContoursHorizontally(self, contour, color):
        """split the drawing space into even rows given a specified row height
            RowHeight = row height in mm
        """
        split_contours= []

        if len(contour) == 0:
            return []
        
        # if the bounding region of contour falls in the row, no need to split
        x,y,w,h = cv2.boundingRect(contour)
        if int(y // self.RowHeight) == int((y+h)// self.RowHeight):
            split_contours.append({'color': color,
                                    'contour': contour})
            return split_contours

        # convert contour to points
        points = contour.reshape(-1,2)

        # add points to the current row
        row_points = []
        last_point = points[-1]


        for i in range(len(points) - 1):
            p1 = points[i]      # current point
            p2 = points[i+1]    # next point
            
            # add current point to current row and check if next point is in a different row
            row_points.append(p1)
            
            row_diff = abs(int(p2[1] // self.RowHeight) - int(p1[1] // self.RowHeight))
            if (row_diff == 0):
                continue
            if (row_diff == 1 and (p2[1] % self.RowHeight ==0) and (p1[1] % self.RowHeight == 0)):
                continue
            

            goDown = p2[1] > p1[1]

            for j in range(row_diff):
                if goDown: # points going down, horizontal cross section below p1
                    y_coord = (self.RowHeight * int(p1[1] // self.RowHeight)) + self.RowHeight
                else: # points going up, horizontal cross section above p1
                    y_coord = (self.RowHeight* int(p2[1]//self.RowHeight)) + ((row_diff-j) * self.RowHeight)

                # find the intersection point on the cross section: y = mx + b
                if (abs(p2[0] - p1[0]) > .01):
                    slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
                    b = p1[1] - (slope*p1[0])
                    x_coord = float((y_coord - b) / slope)
                else: # vertical line
                    x_coord = p1[0]

                # add intersection point to current row and append to the list of split contours
                intersect = [x_coord,y_coord]
                row_points.append(intersect)
                split_contour = np.array(row_points, dtype=np.float32).reshape(-1, 1, 2)
                split_contours.append({'color': color, 
                                    'contour': split_contour})

                # start the next row with intersection point    
                row_points = [intersect]
                p1 = intersect 
            
        
        # add last point to current row and append it to the split_contours
        row_points.append(last_point)
        split_contour = np.array(row_points, dtype=np.float32).reshape(-1, 1, 2)
        split_contours.append({'color': color,
                            'contour': split_contour})

        return split_contours
    

    def remove_path_retrace(self, contours):
        # loop through all paths and find ones that retraces itself
        for i in range(len(contours)):
            points = contours[i]['contour'].reshape(-1, 2)
            n = len(points)
            if n > 3:
                half = int(n // 2)
                # uses symmetry to get rid of dupicate points in the back
                for j in range(half):
                    x_dist = abs(points[j][0] - points[n-1-j][0])
                    y_dist = abs(points[j][1] - points[n-1-j][1])
                    if (x_dist > 3 or y_dist > 3): # guessing expo marker resolution
                        break
                if j>2:  # only removes if detected more than 1 duplicate
                    points = points[:(n-j)]

            contours[i]['contour'] = points.reshape(-1, 1, 2) 
        return contours
   
    def image_to_gcode(self, image_path, output_file1, output_file2, output_file3):
        """write gcode to output file"""
        # detected_words = detect_text(image_path)
        # image_with_text = transpose_print_text_to_image(detected_words, image_path)
        # image_text_path = image_path+"_text_overlay.png"
        # cv2.imwrite(image_text_path, image_with_text)
        # process the image to get clean contours

      #  image_text_path = image_path

        # process the image and get a binary, skeletonized image
        if self.isDigital:
            skeleton_img, binary, org_img= self.preprocess_digital(image_path)
        else:
            skeleton_img, binary, org_img = self.preprocess_photo(image_path)


        # Find contours using RETR_TREE (detects nested contours)
        skeleton_contours, _ = cv2.findContours(skeleton_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
        # detect skeletonized closed shapes and convert them to perfect shapes
        skeleton_contours = self.findShapes(binary, skeleton_contours)

        # filter out small contours
        contours = self.filters_contours(skeleton_contours, skeleton_img)
   
        # remove similar contours
        contours = self.remove_contour_duplicates(contours)

        # assign colors to all contours
        drawing_colors = self.digital_colors if self.isDigital else self.photo_colors
        color_contours = assignColors(org_img, contours, drawing_colors, self.DEFAULT_COLOR)

        # TO DIVIDE CONTOURS INTO ROWS
        split_contours = []
        for i, item in enumerate(color_contours):
            contour = item['contour']
            color = item['color']
            # simplify path by filtering out points
            points = self.simplify_points(contour).astype(np.float32)

            # scale the pixels to millimeters 
            scale_x = self.WIDTH_MM / self.WIDTH_PIXELS
            scale_y = self.HEIGHT_MM / self.HEIGHT_PIXELS
            points[:, 0] *= scale_x
            points[:, 1] *= scale_y

            # split contour into horizontal sections
            contour = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
            split_contours.extend(self.splitContoursHorizontally(contour, color))


        # sort the split contours
        sorted_contours = self.sort_split_contours(split_contours)


        # Open file for writing: file1 for stepper commands, file2 for gear commands, file 3 for plotting
        with open(output_file1, 'w') as f1, open(output_file2, 'w') as f2, open (output_file3,'w') as f3:
            # initial conditions
            current_row = 0 
            LtoR = True
            prev_point = np.array([0,0])
            current_color = None


            for i, item in enumerate(sorted_contours):
                color = item['color']
                contour = item['contour']
                points = contour.reshape(-1, 2)

                if len(points) < 2:
                    continue
   
                # find which row the contour belongs to
                _, y, _, _ = cv2.boundingRect(contour)
                row = y // self.RowHeight    # 0 indexed rows

                if row != current_row:
                    # tell gear motor to move to next row's first point
                    f2.write(f"D{(row - current_row)*self.RowHeight}\n")    # GO DOWN by # of points
                    current_row = row

                # resets direction at the start of every contour by looking at first two points
                if (points[1][0] >= points[0][0]):
                    LtoR = True
                else:
                    LtoR = False
        
                points_gear = points.copy()
                points_gear[:, 0] += self.top_left_x
                points_gear[:, 1] += self.top_left_y

                points_stepper = points.copy()
                points_stepper[:,0] += self.top_left_x
                points_stepper[:,1] -= row*self.RowHeight    # y offset is relative positions in [0,RowHeight]

                # sends color command if color changes
                if (current_color != color):
                    f2.write(f"Color {self.draw_positions[color]}\n")
                    current_color = color

                # Move to first point (marker up)
                f1.write(f"G0 X{points_stepper[0][0]:.2f} Y{points_stepper[0][1]:.2f}\n")
                f2.write(f"G0 X{points_gear[0][0]:.2f} Y{points_gear[0][1]:.2f}\n")
                f3.write(f"G0 X{points[0][0]:.2f} Y{points[0][1]:.2f}\n")

                f3.write(f"Color {color}\n")

                # for plotting original points
                for point in points[1:]:
                    f3.write(f"G1 X{point[0]:.2f} Y{point[1]:.2f}\n")

                prev_point = points_gear[1]

                # Marker down, draw the contour
                for i, point in enumerate(points_gear[2:]):
                    # check if from current point to next point is in the same direction as the current direction
                    f1.write(f"G1 X{points_stepper[i+1][0]:.2f} Y{points_stepper[i+1][1]:.2f}\n")

                    if abs(point[0] - prev_point[0]) < 0.01:
                        f2.write(f"G1 X{prev_point[0]:.2f} Y{prev_point[1]:.2f}\n")
                        f1.write("B\n")

                    else:
                        if LtoR:
                            if point[0] - prev_point[0] < 0:
                                LtoR = False  # backpass, add to gear motor command
                                f2.write(f"G1 X{prev_point[0]:.2f} Y{prev_point[1]:.2f}\n")
                                f1.write("B\n")
                            # else: keep traversing through the points
                        else: 
                            if (point[0]-prev_point[0]) > 0:
                                LtoR = True
                                f1.write("B\n")
                                f2.write(f"G1 X{prev_point[0]:.2f} Y{prev_point[1]:.2f}\n")
                    prev_point = point               
                # add last point
                f2.write(f"G1 X{points_gear[-1][0]:.2f} Y{points_gear[-1][1]:.2f}\n")
                f1.write(f"G1 X{points_stepper[-1][0]:.2f} Y{points_stepper[-1][1]:.2f}\n")
                f1.write("B\n")
                prev_point = points_gear[-1]
                f1.write("\n")
                f2.write("\n")
                f3.write("\n")
            f1.write("M30\n")      # end of drawing, program completes
            f2.write("M30\n") 
            f3.write("M30\n")     

        print(f"G-code successfully written to {output_file1}, {output_file2}, and {output_file3}")
        f1.close()
        f2.close()
        f3.close()
        return output_file1, output_file2, output_file3
    
def main():
    input_folder = "backend/uploads"
    output_folder = "backend/color_output"
    src_file = "rectangle.png"
    gcode_file1 = src_file.rsplit('.', 1)[0] + "1.gcode" #stepper motor commands
    gcode_file2 = src_file.rsplit('.', 1)[0] + "2.gcode" #gear motor commands
    gcode_file3 = src_file.rsplit('.', 1)[0] + "3.gcode" #gear motor commands
    gcode_plotfile = src_file.rsplit('.', 1)[0] + ".png"
    color_map = {
        'black': 'black',
        'blue': 'blue',
        'red': 'black',
        'green': 'blue',
        'purple': 'black',
        'orange': 'blue',
        'brown': 'red',
        'yellow': 'red'
    }

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    converter = GcodeConverter(1100, 525, 'photo', color_map)
    # try:
    image_path = f"{input_folder}/{src_file}" 
    output_file1, output_file2, output_file3 = converter.image_to_gcode(image_path, f"{output_folder}/{gcode_file1}", f"{output_folder}/{gcode_file2}", f"{output_folder}/{gcode_file3}" )
    print(f"G-code saved to: {output_file1} and {output_file2}")
    visualize_gcode(output_file3, f"{output_folder}", gcode_plotfile, 1100, 525)

if __name__ == "__main__":
    main()
    