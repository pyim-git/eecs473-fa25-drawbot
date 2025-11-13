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


digital_colors = [
    'black',
    'blue',
    'red',
    'green',
    'purple',
    'orange',
    'brown',
    'yellow',
    'gray'
]

# no gray
photo_colors = [
    'black',
    'blue',
    'red',
    'green',
    'purple',
    'orange',
    'brown',
    'yellow',
]

# user configurable default color for undetected colors
DEFAULT_COLOR = 'pink' 



def getColorMasks(image, drawing_colors):
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



def assignColors(org_img, contours, isDigital):
    """Loop through contours and assign colors"""

    if isDigital:
        drawing_colors = digital_colors
    else:
        drawing_colors = photo_colors

    color_masks, _= getColorMasks(org_img, drawing_colors)
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
  
    # gray colors gets treated as black
    if best_color == 'gray':
        best_color = 'black'

    return best_color if best_color else 'unknown'
    

                



def splitContoursHorizontally(contour, color, split_y):
    """split the drawing space into even rows given a specified row height

        split_y = row height in mm

        scale_factor = pixels/mm 
    """
    split_contours= []

    if len(contour) == 0:
        return []
    
    # if the bounding region of contour falls in the row, no need to split
    x,y,w,h = cv2.boundingRect(contour)
    if int(y // split_y) == int((y+h)// split_y):
        split_contours.append({'color':color,
                                'contour': contour})
        return split_contours

    # convert contour to points
    points = contour.reshape(-1,2)
   
    #breakpoint()
    # find total rows to get number of contours after splitting
    first_row = int(y // split_y)
    total_rows = math.ceil((y+h)/ split_y) - first_row

    # add points to the current row
    row_points = []
    last_point = points[-1]

    for i in range(len(points) - 1):
        p1 = points[i]      # current point
        p2 = points[i+1]    # next point
        
        # add current point to current row and check if next point is in a different row
        row_points.append(p1)
        row_diff = abs(int(p2[1] // split_y) - int(p1[1] // split_y))
        print(row_diff)
        if (row_diff == 0):
            continue
        if (row_diff == 1 and (p2[1] % split_y ==0) and (p1[1] % split_y == 0)):
            continue

        goDown = p2[1] > p1[1]

        for j in range(row_diff):
            if goDown: # points going down, horizontal cross section below p1
                y_coord = (split_y * int(p1[1] // split_y)) + split_y
            else: # points going up, horizontal cross section above p1
                y_coord = (split_y* int(p2[1]//split_y)) + ((row_diff-j) * split_y)

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
    