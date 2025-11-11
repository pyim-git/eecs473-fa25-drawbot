from paddleocr import PaddleOCR
import cv2
import json
from shapely.geometry import Polygon
import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import math


# ocr = PaddleOCR(
#     use_doc_orientation_classify=False, 
#     use_doc_unwarping=False, 
#     use_textline_orientation=False) # text detection + text recognition

def detect_text(image_path, INPUT_WIDTH=1500, INPUT_HEIGHT=1500, confidence_threshold=0.8):
    # result = ocr.predict(image_path)
    # for res in result:
    #     res.save_to_json("backend/output")
    sub_path = image_path.split('/')[-1]
    json_path = 'backend/output/' + sub_path.replace('.png', '').replace('.jpg', '') + '_res.json'
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
    background = Image.open(image_path).convert("RGBA")
    for text_bbox in detected_words:
        word = detected_words[text_bbox] 
        # Compute box geometry
        width = int(((text_bbox[1][0]-text_bbox[0][0])**2 + (text_bbox[1][1]-text_bbox[0][1])**2)**0.5)
        height = int(((text_bbox[1][0]-text_bbox[2][0])**2 + (text_bbox[1][1]-text_bbox[2][1])**2)**0.5)
        height_from_baseline = text_bbox[3][1] - text_bbox[2][1]
        angle = math.degrees(math.asin(height_from_baseline/width))
        # Create text layer
        text_layer = Image.new("RGBA", (width, height), (255, 255, 255, 255)) #change last arg to 0 to see overlay of original text
        draw = ImageDraw.Draw(text_layer)
        # --- Dynamic font sizing ---
        font_size = int(height * 0.75)
        font = ImageFont.truetype("RobotoMono-Regular.ttf", font_size)
        text_width = draw.textlength(word, font=font)

        while text_width > width * 0.9 and font_size > 5:
            font_size -= 1
            font = ImageFont.truetype("RobotoMono-Regular.ttf", font_size)
            text_width = draw.textlength(word, font=font)
        # ----------------------------
        # centered text
        draw.text((width//2, height//2), word, anchor="mm", font=font, fill="black")
        # rotate about the center
        rotated_text_layer = text_layer.rotate(angle, expand=False)
        background.paste(rotated_text_layer, (text_bbox[0][0], text_bbox[0][1]), rotated_text_layer)
    background.save("backend/output/text.png")


words = detect_text("backend/uploads/myimage.png")
transpose_print_text_to_image(words, "backend/uploads/myimage.png")