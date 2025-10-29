import cv2
import numpy as np
import math
import pdb

# HSV components have different weights for each color
hsv_weights = {
    'black': (0,0.2,0.8),
    'gray': (0,0.7,0.3),
    'red':  (0.8, 0.15, 0.05),
    'blue': (0.7, 0.2, 0.1),
    'green': (0.6, 0.3, 0.1),
    'purple': (0.6, 0.35, 0.05),
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
        'lower1': (0, 90, 90), 'upper1': (10, 255, 255),
        'lower2': (160, 90, 90), 'upper2': (180, 255, 255)   
    },
    'orange': {
        'lower': (10, 80, 80),
        'upper': (30,255,255)
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
    },
    'brown': {
        'lower': (10, 100, 20),
        'upper': (30,255,200)
    }
}


# user configurable drawing colors (at most 3)
drawing_colors = [
    'black',
    'blue',
    'red',
    'green',
    'purple',
    'orange',
    'brown'
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
    color_masks, _= getColorMasks(org_img)
    contour_colors = []

    for i, contour in enumerate(contours):
        color = detectColor(org_img, contour, color_masks)
        if color == 'unknown':
            color = DEFAULT_COLOR
        contour_colors.append({ 
            'color': color,
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
  
    return best_color if best_color else 'unknown'
    

                

def splitContoursHorizontally(points, top_y, bottom_y, split_y = 100):
    """split the drawing space into even rows given a specified row height"""

    # if the bounding region of the contour falls in the row, no need to split
    if int(top_y // split_y) == int(bottom_y // split_y):
        return [points]
    
    if len(points) == 0:
        return[points]
    #breakpoint()
    # find total rows to get number of contours after splitting
    first_row = int(top_y // split_y)
    total_rows = int(math.ceil(bottom_y/ split_y) - first_row)
    split_contours= [[] for _ in range(total_rows)]

    # add points to the current row
    row_points = []
    last_point = points[-1]
    current_index = int(points[0][1] // split_y) - first_row


    for i in range(len(points) - 1):
        p1 = points[i]      # current point
        p2 = points[i+1]    # next point

        # add current point to current row and check if next point is in a different row
        row_points.append(p1)
        row_diff = abs(int(p2[1] // split_y) - int(p1[1] // split_y))
        if (row_diff == 0 ): 
            continue

        goDown = p2[1] > p1[1]


        for j in range(row_diff):
            if goDown: # points going down, horizontal cross section below p1
                y_coord = split_y * int(p1[1] / split_y) + split_y
            else: # points going up, horizontal cross section above p1
                y_coord = split_y * int(p1[1] // split_y)

            # find the intersection point on the cross section: y = mx + b
            if ( (p2[0] - p1[0]) != 0):
                slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
                b = p1[1] - (slope*p1[0])
                x_coord = (y_coord - b) / slope
            else:
                x_coord = p1[0]

            # add intersection point to current row and append to the list of split contours
            intersect = [x_coord,y_coord]
            row_points.append(intersect)
            split_contours.append(row_points)

            # start the next row with intersection point    
            row_points = [intersect] 
            p1 = intersect 
            # if goDown:
            #     current_index = min(current_index + 1, total_rows - 1)
            # else:
            #     current_index = max(current_index - 1, 0)
            current_index = current_index + 1 if goDown else current_index -1
     
    # add last point to current row and append it to the split_contours
    row_points.append(last_point)
    split_contours.append(row_points)
  


    return split_contours
    

        

















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
    