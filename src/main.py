import cv2
import numpy as np
import pytesseract
from PIL import Image, ImageDraw, ImageFont
import os
import subprocess
from visual import *




# Setup directories
input_folder = "../data"
output_folder = "../output"
src_file = "shapes.png"
output = "gcode.out"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)



def intelligent_shape_detection_with_ocr(image_path):
    # Load image
    img = cv2.imread(image_path)
    img_original = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Preprocessing
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    # Morphological operations to clean up
    kernel = np.ones((3,3), np.uint8)
    cleaned = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
    
    # Lists to store different elements
    circles_data = []
    boxes_data = []
    text_regions = []
    ocr_results = []
    
    # Step 1: Detect Circles using HoughCircles
    circles = cv2.HoughCircles(
        gray, 
        cv2.HOUGH_GRADIENT, 
        dp=1.2, 
        minDist=30,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=100
    )
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            circles_data.append((x, y, r))
            cv2.circle(img, (x, y), r, (0, 255, 0), 2)
            cv2.putText(img, "CIRCLE", (x-30, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # Create a mask for non-circular elements
    circle_mask = np.zeros_like(cleaned)
    for (x, y, r) in circles_data:
        cv2.circle(circle_mask, (x, y), r + 5, 255, -1)
    
    # Remove circles from the image for contour detection
    without_circles = cleaned.copy()
    without_circles[circle_mask == 255] = 0
    
    # Step 2: Find Contours for boxes and other shapes
    contours, _ = cv2.findContours(without_circles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 100:
            continue
            
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        
        if len(approx) == 4:
            # It's a quadrilateral - likely a box
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)
            
            if 0.7 < aspect_ratio < 1.3 or (w > 20 and h > 20):
                boxes_data.append(approx)
                cv2.drawContours(img, [approx], -1, (255, 0, 0), 2)
                
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.putText(img, "BOX", (cx-15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        else:
            # Potential text region
            x, y, w, h = cv2.boundingRect(contour)
            
            if (w > 15 and h > 15) and (h < 100 and w < 300):
                contour_area = cv2.contourArea(contour)
                rect_area = w * h
                extent = float(contour_area) / rect_area if rect_area > 0 else 0
                
                if extent < 0.6:
                    text_regions.append((x, y, w, h))
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
    
    # Step 3: OCR Processing on text regions
    print("Performing OCR on text regions...")
    
    # For OCR, we need the original image (not inverted)
    _, thresh_ocr = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    for i, (x, y, w, h) in enumerate(text_regions):
        # Extract the text region with padding
        padding = 5
        x1 = max(0, x - padding)
        y1 = max(0, y - padding)
        x2 = min(img.shape[1], x + w + padding)
        y2 = min(img.shape[0], y + h + padding)
        
        text_roi = thresh_ocr[y1:y2, x1:x2]
        
        if text_roi.size == 0:
            continue
            
        # Configure OCR for better text recognition
        custom_config = r'--oem 3 --psm 8 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789'
        
        try:
            # Perform OCR
            detected_text = pytesseract.image_to_string(text_roi, config=custom_config)
            detected_text = detected_text.strip()
            
            if len(detected_text) > 0:
                print(f"Region {i+1}: '{detected_text}'")
                
                ocr_results.append({
                    'text': detected_text,
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })
                
                # Draw the recognized text on the image
                cv2.putText(img, f"TEXT: {detected_text}", (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                cv2.putText(img, "TEXT: [No OCR]", (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
        except Exception as e:
            print(f"OCR error in region {i+1}: {e}")
            cv2.putText(img, "TEXT: [OCR Error]", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    # Display results
    cv2.imshow('Original', img_original)
    cv2.imshow('Processed', cleaned)
    cv2.imshow('Intelligent Detection with OCR', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return {
        'circles': circles_data,
        'boxes': boxes_data,
        'text_regions': text_regions,
        'ocr_results': ocr_results,
        'image_size': img.shape
    }

def text_to_gcode_vector(text, position, font_size=10, spacing=1.2):
    """
    Convert text to G-code using vector font rendering
    This creates actual vector paths for each character
    """
    gcode_commands = []
    
    # Create a PIL image to draw text and extract vector points
    pil_img = Image.new('L', (800, 200), 255)  # Large canvas for text
    draw = ImageDraw.Draw(pil_img)
    
    try:
        # Try to use a vector-friendly font
        font = ImageFont.truetype("arial.ttf", font_size)
    except:
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", font_size)
        except:
            # Fallback to default font
            font = ImageFont.load_default()
    
    # Get text bounding box
    bbox = draw.textbbox((0, 0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    
    # Draw text
    draw.text((10, 10), text, font=font, fill=0)
    
    # Convert to OpenCV for contour detection
    cv_img = np.array(pil_img)
    _, thresh = cv2.threshold(cv_img, 127, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours of the text
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    gcode_commands.append(f"; Text: '{text}'")
    gcode_commands.append("G0 Z5 ; Lift tool")
    
    # Scale factor and offset to position text correctly
    scale_x = 0.1  # Adjust based on your needs
    scale_y = 0.1
    offset_x = position[0] - (text_width * scale_x / 2)
    offset_y = position[1] - (text_height * scale_y / 2)
    
    for i, contour in enumerate(contours):
        if cv2.contourArea(contour) < 5:  # Filter noise
            continue
            
        # Simplify contour
        epsilon = 0.007 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Convert to G-code path
        points = approx.reshape(-1, 2)
        
        if len(points) > 1:
            # Move to first point
            gcode_x = offset_x + (points[0][0] - 10) * scale_x
            gcode_y = offset_y + (points[0][1] - 10) * scale_y
            gcode_commands.append(f"G0 X{gcode_x:.2f} Y{gcode_y:.2f}")
            gcode_commands.append("G1 Z0 ; Lower tool")
            
            # Draw the character
            for point in points[1:]:
                gcode_x = offset_x + (point[0] - 10) * scale_x
                gcode_y = offset_y + (point[1] - 10) * scale_y
                gcode_commands.append(f"G1 X{gcode_x:.2f} Y{gcode_y:.2f}")
            
            # Close the contour if needed
            gcode_x = offset_x + (points[0][0] - 10) * scale_x
            gcode_y = offset_y + (points[0][1] - 10) * scale_y
            gcode_commands.append(f"G1 X{gcode_x:.2f} Y{gcode_y:.2f}")
            
            gcode_commands.append("G0 Z5 ; Lift tool")
    
    return gcode_commands

def generate_gcode_with_ocr(detection_results):
    """Convert detected shapes and OCR text to G-code"""
    gcode_commands = []
    
    # G-code header
    gcode_commands.append("; G-code generated from intelligent OCR detection")
    gcode_commands.append("G21 ; Millimeter units")
    gcode_commands.append("G90 ; Absolute positioning")
    gcode_commands.append("G0 Z5 ; Lift tool")
    
    # Process circles
    print(f"Found {len(detection_results['circles'])} circles")
    for i, (x, y, r) in enumerate(detection_results['circles']):
        gcode_commands.append(f"; Circle {i+1} at ({x}, {y}) radius {r}")
        gcode_commands.append(f"G0 X{x} Y{y}")
        gcode_commands.append("G1 Z0")
        gcode_commands.append(f"G1 X{x+r} Y{y}")
        gcode_commands.append(f"G3 X{x+r} Y{y} I{-r} J0")
        gcode_commands.append("G0 Z5")
    
    # Process boxes
    print(f"Found {len(detection_results['boxes'])} boxes")
    for i, box in enumerate(detection_results['boxes']):
        gcode_commands.append(f"; Box {i+1}")
        points = box.reshape(-1, 2)
        points = np.vstack([points, points[0]])
        
        gcode_commands.append(f"G0 X{points[0][0]} Y{points[0][1]}")
        gcode_commands.append("G1 Z0")
        for point in points[1:]:
            gcode_commands.append(f"G1 X{point[0]} Y{point[1]}")
        gcode_commands.append("G0 Z5")
    
    # Process OCR text with vector fonts
    print(f"Found {len(detection_results['ocr_results'])} text elements via OCR")
    for i, text_data in enumerate(detection_results['ocr_results']):
        text = text_data['text']
        center_x, center_y = text_data['center']
        
        gcode_commands.append(f"; OCR Text {i+1}: '{text}'")
        
        # Generate vector G-code for the text
        text_gcode = text_to_gcode_vector(text, (center_x, center_y), font_size=20)
        gcode_commands.extend(text_gcode)
    
    # G-code footer
    gcode_commands.append("G0 Z10")
    gcode_commands.append("G0 X0 Y0")
    gcode_commands.append("M2")
    
    return gcode_commands



# Main execution
if __name__ == "__main__":
    
    
    # Use your image or create a sample
    image_path = f"{input_folder}/{src_file}"  # Replaces with your image path
    gcode_file = f"{output_folder}/{output}"
    
    if not os.path.exists(image_path):
        print("Image not found")
    
    # Detect shapes and perform OCR
    print("Starting intelligent shape detection with OCR...")
    results = intelligent_shape_detection_with_ocr(image_path)
    
    # Generate G-code
    print("Generating G-code with vector text...")
    gcode = generate_gcode_with_ocr(results)
    
    # Save to file
    with open("ocr_vector_output.gcode", "w") as f:
        for line in gcode:
            f.write(line + "\n")
    
    print("\n=== PROCESSING COMPLETE ===")
    print(f"Circles: {len(results['circles'])}")
    print(f"Boxes: {len(results['boxes'])}")
    print(f"Text regions: {len(results['text_regions'])}")
    print(f"OCR results: {len(results['ocr_results'])}")
    
    if results['ocr_results']:
        print("\nRecognized text:")
        for i, ocr_data in enumerate(results['ocr_results']):
            print(f"  {i+1}. '{ocr_data['text']}'")
    
    print(f"\nG-code saved to: ocr_vector_output.gcode")