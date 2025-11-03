import numpy as np


def simplify_line(contours, max_angle_deviation=10):
    """
    Simple method: remove points that don't significantly change direction
    """
    simplified_contours = []
    
    for contour in contours:
        points = contour.reshape(-1, 2)
        
        if len(points) < 3:
            simplified_contours.append(contour)
            continue
        
        # Keep points where direction changes significantly
        keep_indices = [0]  # Always keep first point
        
        for i in range(1, len(points)-1):
            # Calculate vectors to previous and next points
            v1 = points[i] - points[i-1]
            v2 = points[i+1] - points[i]
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                # Normalize vectors
                v1 = v1 / np.linalg.norm(v1)
                v2 = v2 / np.linalg.norm(v2)
                
                # Calculate angle between vectors (in degrees)
                angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0)))
                
                # Keep point if direction changes significantly
                if angle > max_angle_deviation:
                    keep_indices.append(i)
        
        keep_indices.append(len(points)-1)  # Always keep last point
        
        simplified_points = points[keep_indices]
        simplified_contour = simplified_points.reshape(-1, 1, 2).astype(np.int32)
        simplified_contours.append(simplified_contour)
        
        print(f"Straight line simplification: {len(points)} -> {len(simplified_points)} points")
    
    return simplified_contours