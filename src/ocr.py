from paddleocr import PaddleOCR
import cv2
import json
from shapely.geometry import Polygon
import os
from font_contours import *

ocr = PaddleOCR(
    use_doc_orientation_classify=False, 
    use_doc_unwarping=False, 
    use_textline_orientation=False) # text detection + text recognition

contour_lib = ContourLibrary()

def detect_text(image_path, INPUT_WIDTH=1500, INPUT_HEIGHT=1500, confidence_threshold=0.9):
    result = ocr.predict(image_path)
    for res in result:
        res.save_to_img("output")
        res.save_to_json("output")
    sub_path = image_path.split('/')[-1]
    json_path = 'output/' + sub_path.replace('.png', '').replace('.jpg', '') + '_res.json'
    with open(json_path, 'r') as file:
        result = json.load(file)
    confidence = result["rec_scores"]
    words = dict()
    for i in range(len(confidence)):
        if (confidence[i] > confidence_threshold):
            text_bbox = [(point[0], point[1]) for point in result["rec_polys"][i]]
            words[tuple(text_bbox)] = result["rec_texts"][i]
    return words

def transpose_print_text_to_image(detected_words, image_path):
    """Overlay detected text bounding boxes on the original image for visualization"""
    image = cv2.imread(image_path)
    sub_path = image_path.split('/')[-1]
    image_text_path = 'output/' + sub_path.replace('.png', '').replace('.jpg', '') + '_ocr_res_img.png'
    image_text = cv2.imread(image_text_path)
    #only use the right half of the image
    image_text = image_text[:, image_text.shape[1] // 2:]
    for text_bbox in detected_words:
       # Create a blank mask
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        # Points of your polygon
        text_bbox_zoomed_in = []
        for i, coord in enumerate(text_bbox):
            if i==0:
                text_bbox_zoomed_in.append((coord[0]+2, coord[1]+2))
            elif i==1:
                text_bbox_zoomed_in.append((coord[0]-2, coord[1]+2))
            elif i==2:
                text_bbox_zoomed_in.append((coord[0]-2, coord[1]-2))
            else:
                text_bbox_zoomed_in.append((coord[0]+2, coord[1]-2))
        points = np.array(text_bbox_zoomed_in, dtype=np.int32)
        # Fill mask for polygon region
        cv2.fillPoly(mask, [points], 255)
        # Extract that region from image_text
        region = cv2.bitwise_and(image_text, image_text, mask=mask)
        # Also mask out region in original image
        image_bg = cv2.bitwise_and(image, image, mask=cv2.bitwise_not(mask))
        # Combine them: region from text-image goes onto base-image
        image = cv2.add(image_bg, region)
    return image

words = detect_text("color_img/mytest.png")
transpose_print_text_to_image(words, "color_img/mytest.png")
# remove_contours_in_text_boxes_bbox([], words)
