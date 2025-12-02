import cv2
import numpy as np
import math
import pdb



# HSV ranges for each color
hsv_ranges = {
    'black': {
        'lower': (0,0,0),
        'upper':(180,60,50)
    },
    'gray': {
        'lower': (0,0,50),
        'upper': (180, 40, 200)
    },
    'red':{
        'lower1': (0, 100, 100), 'upper1': (10, 255, 255),
        'lower2': (160, 80, 80), 'upper2': (180, 255, 255)   
    },
    'orange': {
        'lower': (10, 80, 80),
        'upper': (20,255,255)
    },
    'yellow': {
        'lower': (20,80,80),
        'upper': (40, 255,255)
    },
    'blue': {
        'lower': (95,100,50),
        'upper': (125, 255,255)
    },
    'green': {
        'lower': (30, 50,50),
        'upper': (95, 255, 255),
    },
    'purple': {
        'lower': (125, 50, 30),
        'upper': (160, 255, 255)
    },
    'brown': {
        'lower': (5, 60, 20),
        'upper': (25,200,120)
    }
}



def getColorMasks(image, drawing_colors):
    """Create masks showing where each color appears in the entire image"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    color_masks = {}

    # create color masks for every drawing color
    for color in drawing_colors:
        if color == 'red':
            mask1 = cv2.inRange(hsv, 
                              np.array(hsv_ranges['red']['lower1']), 
                              np.array(hsv_ranges['red']['upper1']))
            mask2 = cv2.inRange(hsv,
                              np.array(hsv_ranges['red']['lower2']), 
                              np.array(hsv_ranges['red']['upper2']))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv,
                            np.array(hsv_ranges[color]['lower']), 
                            np.array(hsv_ranges[color]['upper']))

        # clean the mask in the same way as with image preprocessing 
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        # get the detected pixel count for each color 
        color_masks[color] = mask
            
    return color_masks



def assignColors(org_img, contours, drawing_colors, default_color):
    """Loop through contours and assign colors"""


    color_masks = getColorMasks(org_img, drawing_colors)
    contour_colors = []

    for contour in contours:
        color = detectColor(org_img, contour, color_masks)
        if color == 'unknown':
            color = default_color
        contour_colors.append({ 
            'color': drawing_colors[color],
            'contour': contour
        })
   
    return contour_colors


def detectColor(image, contour, color_masks, stroke_width =30): 
    """adjustable stroke width for better sampling"""

    # Find best color match by overlaping contour mask with color mask
    contour_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, thickness=stroke_width) 
    
    best_color = None
    best_area = 0
    
    for color_name, color_mask in color_masks.items():
        overlap = cv2.bitwise_and(contour_mask, color_mask)
        overlap_area = np.sum(overlap == 255)

        if overlap_area > best_area:
            best_area = overlap_area
            best_color = color_name
  
    # gray colors gets treated as black
    if best_color == 'gray':
        best_color = 'black'

    return best_color if best_color else 'unknown'
    

            

