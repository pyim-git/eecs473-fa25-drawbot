import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import os

class DrawingStandardizer:
    def __init__(self, line_tolerance=5, circle_tolerance=0.1):
        self.line_tolerance = line_tolerance
        self.circle_tolerance = circle_tolerance
    
    def standardize_image(self, image_path):
        """Standardize image with strict circle detection"""
        image = cv2.imread(image_path)
        if image is None:
            print("ERROR: Could not load image!")
            return None
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Found {len(contours)} contours")
        
        standardized_elements = []
        
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < 50:
                continue
                
            element_type = self.classify_element_strict(contour)
            print(f"Contour {i}: {element_type}")
            
            if element_type == "circle":
                standardized = self.standardize_circle(contour)
            elif element_type == "line":
                standardized = self.standardize_line(contour)
            elif element_type == "rectangle":
                standardized = self.standardize_rectangle(contour)
            else:
                standardized = self.standardize_character(contour)
            
            standardized_elements.append({
                'type': element_type,
                'points': standardized,
                'original': contour
            })
        
        self.visualize_results(image, binary, standardized_elements)
        return standardized_elements
    
    def classify_element_strict(self, contour):
        """STRICT circle detection - rejects polygons"""
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0:
            return "character"
        
        # Basic circularity
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # Get enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        enclosing_area = np.pi * radius * radius
        
        # Circle fit ratio
        circle_fit = area / enclosing_area if enclosing_area > 0 else 0
        
        # Polygon approximation to detect vertices
        epsilon = 0.02 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        
        # STRICT CIRCLE CRITERIA:
        is_circle = (
            circularity > 0.85 and          # Very circular
            circle_fit > 0.75 and           # Fits enclosing circle well
            num_vertices > 10 and           # Many vertices (not a polygon)
            area > 100                      # Not too small
        )
        
        if is_circle:
            return "circle"
        
        # Check for other shapes
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / h if h > 0 else 0
        
        # Rectangle detection
        rect_area = w * h
        extent = area / rect_area if rect_area > 0 else 0
        if extent > 0.7 and (0.3 < aspect_ratio < 3.0):
            return "rectangle"
        
        # Line detection
        if max(w, h) / min(w, h) > 4:
            return "line"
        
        return "character"
    
    def standardize_circle(self, contour):
        """Create perfect circle"""
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        
        # Generate smooth circle
        angles = np.linspace(0, 2 * np.pi, 72)  # More points for smoother circle
        circle_points = []
        for angle in angles:
            px = center[0] + radius * np.cos(angle)
            py = center[1] + radius * np.sin(angle)
            circle_points.append([px, py])
        
        return np.array(circle_points, dtype=np.int32)
   
    def standardize_line(self, contour):
        """Create straight line"""
        points = contour.reshape(-1, 2)
        
        if len(points) < 2:
            return points
        
        # Fit line
        [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        line_dir = np.array([vx[0], vy[0]])
        line_point = np.array([x[0], y[0]])
        
        # Project points to find endpoints
        projections = []
        for point in points:
            vec = point - line_point
            projection = np.dot(vec, line_dir)
            projections.append(projection)
        
        start_proj = min(projections)
        end_proj = max(projections)
        
        start_point = line_point + start_proj * line_dir
        end_point = line_point + end_proj * line_dir
        
        # Create straight line
        t_values = np.linspace(0, 1, 10)
        straight_points = []
        for t in t_values:
            point = start_point + t * (end_point - start_point)
            straight_points.append([int(point[0]), int(point[1])])
        
        return np.array(straight_points, dtype=np.int32)
    
    def standardize_rectangle(self, contour):
        """Create perfect rectangle"""
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        return box
    
    def standardize_character(self, contour):
        """Simplify character contour"""
        epsilon = 0.02 * cv2.arcLength(contour, True)
        simplified = cv2.approxPolyDP(contour, epsilon, True)
        return simplified.reshape(-1, 2)
    
    def visualize_results(self, original_image, binary_image, standardized_elements):
        """Show before/after comparison"""
        # Create clean output
        clean_image = np.ones_like(original_image) * 255
        
        # Draw standardized elements
        for element in standardized_elements:
            points = element['points']
            color = (0, 0, 0)  # Black
            
            if element['type'] == 'circle':
                cv2.polylines(clean_image, [points], True, color, 2)
            elif element['type'] == 'line':
                cv2.polylines(clean_image, [points], False, color, 2)
            else:
                cv2.polylines(clean_image, [points], True, color, 2)
        
        # Convert to RGB for matplotlib
        original_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
        clean_rgb = cv2.cvtColor(clean_image, cv2.COLOR_BGR2RGB)
        
        # Plot results
        plt.figure(figsize=(15, 5))
        
        plt.subplot(1, 3, 1)
        plt.imshow(original_rgb)
        plt.title('Original Image')
        plt.axis('off')
        
        plt.subplot(1, 3, 2)
        plt.imshow(binary_image, cmap='gray')
        plt.title('Binary (Contours)')
        plt.axis('off')
        
        plt.subplot(1, 3, 3)
        plt.imshow(clean_rgb)
        plt.title('Standardized Drawing')
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()
        
        print(f"Standardized {len(standardized_elements)} elements")
        for element in standardized_elements:
            print(f"  - {element['type']}: {len(element['points'])} points")

# TEST WITH A SIMPLE EXAMPLE
def create_test_image():
    """Create a test image with circles, lines, and rectangles"""
    img = np.ones((300, 400, 3), dtype=np.uint8) * 255
    
    # Draw a circle
    cv2.circle(img, (100, 100), 30, (0, 0, 0), 2)
    
    # Draw a line
    cv2.line(img, (200, 50), (300, 150), (0, 0, 0), 2)
    
    # Draw a rectangle
    cv2.rectangle(img, (50, 200), (150, 250), (0, 0, 0), 2)
    
    # Draw some text/character
    cv2.putText(img, 'A', (250, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    
    return img


# Setup directories
input_folder = "../data"
output_folder = "../output"
src_file = "shapes.png"
output = "gcode.out"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)


# RUN THE CODE
if __name__ == "__main__":
    # Option 1: Use your image
    standardizer = DrawingStandardizer()
    image_path = f"{input_folder}/{src_file}" 

    result = standardizer.standardize_image(image_path)
    
    # Option 2: Test with generated image
    print("Creating test image...")
    test_image = create_test_image()
    cv2.imwrite("test_drawing.png", test_image)
    
    print("Running standardizer...")
    standardizer = DrawingStandardizer()
    result = standardizer.standardize_image("test_drawing.png")
    
    if result is None:
        print("No result generated. Check the debug output above.")