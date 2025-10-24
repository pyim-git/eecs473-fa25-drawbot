import cv2
import numpy as np


# HSV components have different weights for each color
hsv_weights = {
    'black': (0,0.2,0.8),
    'gray': (0,0.7,0.3),
    'red':  (0.8, 0.15, 0.05),
    'blue': (0.7, 0.2, 0.1),
    'green': (0.6, 0.3, 0.1),
    'purple': (0.6, 0.35, 0.05)
}

# HSV ranges for each color
hsv_ranges = {
    'black': {
        'lower': (0,0,0),
        'upper':(180,50,50)
    },
    'gray': {
        'lower': (0,0,50),
        'upper': (180, 50, 200)
    },
    'red':{
        'lower1': (0, 80, 80), 'upper1': (10, 255, 255),
        'lower2': (160, 80, 80), 'upper2': (180, 255, 255)   
    },
    'blue': {
        'lower': (95,50,30),
        'upper': (130, 255,255)
    },
    'green': {
        'lower': (30, 50,50),
        'upper': (95, 255, 255),
    },
    'purple': {
        'lower': (120, 50, 30),
        'upper': (155, 255, 255)
    }
}


# user configurable drawing colors (at most 3)
drawing_colors = [
    'black',
    'blue',
    'red',
    'green',
    'purple',
]

# user configurable default color for undetected colors
DEFAULT_COLOR = 'black' 



def getColorMasks(image):
    """Create masks showing where each color appears in the entire image"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    color_masks = {}
    color_areas = {}
    result = np.ones_like(image)* 255

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
        pixel_count = np.sum(mask == 255)
        print(f"{color}: {pixel_count} pixels")
            
    return color_masks, result




def assignColors(org_img, contours):
    """Loop through contours and assign colors"""
    color_masks, _,  _= getColorMasks(org_img)
    contour_colors = []

    for i, contour in enumerate(contours):
        color = detectColor(org_img, contour, color_masks)
        if color is 'unknown':
            color = DEFAULT_COLOR
        contour_colors.append({ 
            'color': color,
            'contour': contour
        })
   
    return contour_colors


def detectColor(image, contour, color_masks, stroke_width =20): 
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
  
    return best_color if best_color else 'unknown'
    





# def main():
#     # try:
 
#     #     # image = cv2.imread("../data/colors.png")
#     #     # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
#     #     # image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel, iterations=1)


#     #     # cv2.imshow('ALL DETECTED COLORS', result)
#     #     # cv2.waitKey(0)
#     #     # cv2.destroyAllWindows()


#     # except Exception as e:
#     #     print(f"Error with custom image: {e}")


# if __name__ == "__main__":
#     main()
    