
import subprocess
import os
import cv2
import numpy as np
import pdb
#from sklearn.neighbors import NearestNeighbors
import math
from visual import *
#from simplify_points import *
#from skimage.morphology import skeletonize


# Setup directories
input_folder = "../data"
output_folder = "../output"
src_file = "shapes.png"
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


        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
        
        # clean up noises and blurs
        kernel = np.ones((3,3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        return img, binary
    
    

    """Sorts the contours: Path goes left->right, top->bottom"""
    def optimize_path(self, contours):
        if contours is None or len(contours) == 0:
            return contours
        
        contours_list = list(contours)
        contours_list.sort(key=lambda c: (cv2.boundingRect(c)[1], cv2.boundingRect(c)[0]))
        
        return contours_list

    """Write gcode to output file"""
    def image_to_gcode(self, image, output_file):
        original_img, binary_img = self.preprocess(image)
        
        # Find contours
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        #contours = [c for c in contours if cv2.contourArea(c) > 100]
        #contours = simplify_line(contours)

        # Open file for writing
        with open(output_file, 'w') as f:
            f.write("G90\n")  # Move to origin (0,0) at top left corner 

            # current path order: left to right, top to bottom
            contours = self.optimize_path(contours)
            print(f"Found {len(contours)} contours")

            for i, contour in enumerate(contours):
                # check if the contour is closed or open path. 
                all_points = contour.reshape(-1, 2) 
                points = all_points
                start = all_points[0]
                end = all_points[-1]
                distance = np.linalg.norm(start - end)
                isClosed = False
                if distance < 5.0:  # if distance between start and end is close, close the path
                    isClosed = True
            
                # # simplify the points in the contour (adjustable for shape precision)
                epsilon = 0.002 * cv2.arcLength(contour, True)
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

                if isClosed:  # connect end to start if closed path
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
    










# import subprocess
# import os
# import cv2
# import numpy as np
# import pdb
# #from sklearn.neighbors import NearestNeighbors
# import math



# # Setup directories
# input_folder = "data"
# output_folder = "output"
# src_file = "test_image.png"
# output = "gcode.out"
# if not os.path.exists(output_folder):
#     os.makedirs(output_folder)

# # user-configurable drawing dimension in pixels
# INPUT_WIDTH = 1000 
# INPUT_HEIGHT = 1500

# # RGB thresholds for 3 uniquecolors
# color_thres = {
#     'red': {
#         'lower': [150, 0, 0],     
#         'upper': [255, 50, 50]     
#     },
#     'green': {
#         'lower': [0, 150, 0],     
#         'upper': [50, 255, 50]    
#     },
#     'blue': {
#         'lower': [0, 0, 150],     
#         'upper': [50, 50, 255]    
#     }
# }


# class GcodeConverter:
#     def __init__(self):
#         self.M_UP = "M5"
#         self.M_DOWN = "M3"
   
    
#     """IMAGE PREPROCESSING"""
#     def preprocess_image(self, image):
#         img = cv2.imread(image)
#         if img is None:
#             raise ValueError(f"Could not load image from {image}")

#         # Resize image
#         img = cv2.resize(img, (INPUT_WIDTH, INPUT_HEIGHT))
    

#         # TO_DO: use a mask for processing 3 different color threshold 
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
#         # clean up noises and blurs
#         kernel = np.ones((3,3), np.uint8)
#         binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
#         return img, binary
    
#     """G_CODE GENERATER"""
#     def image_to_gcode(self, image, output_file):
#         original_img, binary_img = self.preprocess_image(image)
        
#         # Find contours
#         contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         contours = [c for c in contours if cv2.contourArea(c) > 100]
#         print(f"Found {len(contours)} contours")
        
#         # Open file for writing
#         with open(output_file, 'w') as f:
#             f.write("G90\n")        # Absolute positioning to startpoint
             
#             # Process each contour and write directly to file
#             for i, contour in enumerate(contours):
#                 print(f"Processing contour {i+1}/{len(contours)}")
#                 self.write_contour_to_file(f, contour, i+1)
            
#             f.write("M30\n")       # end of drawing, program completes
        
#         # Visualize results
#       #  self.visualize_results(original_img, contours, "detected_contours.png")
#         print(f"G-code successfully written to {output_file}")
#         return output_file
    
   
#     """helper function for g-code generation for each contour"""
#     def write_contour_to_file(self, file_handle, contour, contour_number):
        
#         # Add simplication for reducing points in a countour to speed up the robot it needed
#         # Needs to simplify straight lines and leave curves alone, the commented code oversimplify curves which is bad
#        # epsilon = 0.01 * cv2.arcLength(contour, True)
#        # simplified = cv2.approxPolyDP(contour, epsilon, True)
#          # Calculate shape properties for smart simplification
#             # Smart simplification that preserves shapes
#         points = contour.reshape(-1, 2)
#         if len(points) < 2:
#             return
    
#         # Basic simplification - good for pentagons
#         if len(points) > 15:
#             epsilon = 0.02 * cv2.arcLength(contour, True)
#             simplified = cv2.approxPolyDP(contour, epsilon, True)
#             points = simplified.reshape(-1, 2)
            
  
                
        
#         # move to starting position, tool down, and trace
#         file_handle.write(f"Contour {contour_number}\n")
#         file_handle.write(f"G0 X{points[0][0]:.2f} Y{points[0][1]:.2f}\n")
#         file_handle.write(f"{self.M_DOWN}\n")
        
#         # Draw the contour
#         for point in points[1:]:
#             file_handle.write(f"G1 X{point[0]:.2f} Y{point[1]:.2f}\n")
        
#         # done
#         file_handle.write(f"{self.M_UP}\n")
#         file_handle.write("\n")  
    


#     # def visualize_results(self, original_img, contours, output_path):
#     #     """Create visualization image"""
#     #     vis_img = original_img.copy()
#     #     cv2.drawContours(vis_img, contours, -1, (0, 255, 0), 2)
        
#     #     for i, contour in enumerate(contours):
#     #         if len(contour) > 0:
#     #             start_point = tuple(contour[0][0])
#     #             cv2.circle(vis_img, start_point, 5, (0, 0, 255), -1)
        
#     #     cv2.imwrite(output_path, vis_img)
#     #     print(f"Visualization saved as {output_path}")



# def main():
#     converter = GcodeConverter()

#     try:
#         image_path = f"{input_folder}/{src_file}" 
#         output_file = converter.image_to_gcode(image_path, f"{output_folder}/{output}")
#         print(f"G-code saved to: {output_file}")
        
#     except Exception as e:
#         print(f"Error with g-code generation: {e}")


# if __name__ == "__main__":
#     main()