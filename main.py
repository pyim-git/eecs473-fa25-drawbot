
import subprocess
import os
import cv2
import numpy as np
import pdb
#from sklearn.neighbors import NearestNeighbors
import math



# Setup directories
input_folder = "data"
output_folder = "output"
src_file = "diamond.png"
output = "gcode.out"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# user-configurable drawing dimension in pixels
INPUT_WIDTH = 1000 
INPUT_HEIGHT = 1500

# HSV thresholds for 3 uniquecolors
color_thres = {
    'green': {
        'lower': [35, 50, 50],     
        'upper': [85, 255, 255]     
    },
    'black': {
        'lower': [0, 0, 0],      # the HSV values may require adjustment
        'upper': [179, 50, 255]    
    },
    'red': {
        'lower': [100, 50, 50],     
        'upper': [130, 255, 255]    
    }
}

    
class GcodeConverter:
    def __init__(self):
        self.M_UP = "M5"
        self.M_DOWN = "M3"
        self.color_commands = {
            'green': "G",    
            'black': "B",  
            'red': "R"    
        }

    def get_color_contours(self, image):
        """Extract contours for each color"""
        img = cv2.imread(image)
        img = cv2.resize(img, (INPUT_WIDTH, INPUT_HEIGHT))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        color_contours = {}
        
        for color_name, ranges in color_thres.items():
            # Create mask
            if isinstance(ranges, list):
                mask = None
                for r in ranges:
                    range_mask = cv2.inRange(hsv, np.array(r['lower']), np.array(r['upper']))
                    mask = range_mask if mask is None else cv2.bitwise_or(mask, range_mask)
            else:
                mask = cv2.inRange(hsv, np.array(ranges['lower']), np.array(ranges['upper']))

            # find contours in this color
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 100]
            color_contours[color_name] = contours
            
            # Clean mask
            # kernel = np.ones((3,3), np.uint8)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            print(f"Found {len(contours)} contours for {color_name}")
        
        return color_contours, img

    def image_to_gcode(self, image, output_file):
        color_contours, img = self.get_color_contours(image)
        
        with open(output_file, 'w') as f:
            f.write("G90\n")  # Absolute positioning
            
            for color_name, contours in color_contours.items():
                if not contours:
                    continue
                    
                print(f"Processing {len(contours)} contours for {color_name}")
                
                # Color change command
                f.write(f"{self.color_commands[color_name]}\n")
                
                # Process contours
                for i, contour in enumerate(contours):
                    self.write_contour(f, contour, f"{color_name}_{i+1}")

            f.write("M30\n")  # Program end
        
        print(f"G-code written to {output_file}")
        return output_file
    
    def write_contour(self, file_handle, contour, contour_id):
        points = contour.reshape(-1, 2)
        if len(points) < 2:
            return
        
        # Move to start
        file_handle.write(f"G0 X{points[0][0]:.2f} Y{points[0][1]:.2f}\n")
        file_handle.write(f"{self.M_DOWN}\n")
        
        # Draw contour
        for point in points[1:]:
            file_handle.write(f"G1 X{point[0]:.2f} Y{point[1]:.2f}\n")
        
        file_handle.write(f"{self.M_UP}\n\n")
    
 
def main():
    converter = GcodeConverter()
    try:
        image_path = f"{input_folder}/{src_file}" 
        output_file = converter.image_to_gcode(image_path, f"{output_folder}/{output}")
        print(f"G-code saved to: {output_file}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()