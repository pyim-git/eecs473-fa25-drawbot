import numpy as np
import cv2

class ContourLibrary:
    """
    A library of single-line contours for uppercase/lowercase letters and numbers.
    Contours are designed to be compatible with cv2.drawContours().
    """
    
    def __init__(self, height=100):
        """
        Initialize the contour library with a specified character height.
        
        Args:
            height: The nominal height of characters in pixels
        """
        self.height = height
        self.contours = {}
        self._build_library()
    
    def _scale_points(self, points):
        """Scale normalized points (0-1 range) to the character height."""
        return (np.array(points) * self.height).astype(np.int32)
    
    def _build_library(self):
        """Build all character contours."""
        
        # Uppercase letters
        self.contours['A'] = self._scale_points([
            [0.2, 1.0], [0.5, 0.0], [0.8, 1.0],  # Outer shape
            [0.65, 0.6], [0.35, 0.6]  # Crossbar
        ])
        
        self.contours['B'] = self._scale_points([
            [0.2, 0.0], [0.2, 1.0], [0.7, 1.0], [0.8, 0.85], [0.7, 0.5],
            [0.2, 0.5], [0.75, 0.5], [0.85, 0.35], [0.75, 0.0], [0.2, 0.0]
        ])
        
        self.contours['C'] = self._scale_points([
            [0.85, 0.2], [0.7, 0.0], [0.3, 0.0], [0.15, 0.15],
            [0.15, 0.85], [0.3, 1.0], [0.7, 1.0], [0.85, 0.8]
        ])
        
        self.contours['D'] = self._scale_points([
            [0.2, 0.0], [0.2, 1.0], [0.6, 1.0], [0.8, 0.85],
            [0.8, 0.15], [0.6, 0.0], [0.2, 0.0]
        ])
        
        self.contours['E'] = self._scale_points([
            [0.8, 0.0], [0.2, 0.0], [0.2, 0.5], [0.65, 0.5],
            [0.2, 0.5], [0.2, 1.0], [0.8, 1.0]
        ])
        
        self.contours['F'] = self._scale_points([
            [0.2, 1.0], [0.2, 0.5], [0.65, 0.5],
            [0.2, 0.5], [0.2, 0.0], [0.8, 0.0]
        ])
        
        self.contours['G'] = self._scale_points([
            [0.85, 0.2], [0.7, 0.0], [0.3, 0.0], [0.15, 0.15],
            [0.15, 0.85], [0.3, 1.0], [0.7, 1.0], [0.85, 0.85],
            [0.85, 0.55], [0.55, 0.55]
        ])
        
        self.contours['H'] = self._scale_points([
            [0.2, 0.0], [0.2, 1.0], [0.2, 0.5],
            [0.8, 0.5], [0.8, 0.0], [0.8, 1.0]
        ])
        
        self.contours['I'] = self._scale_points([
            [0.3, 0.0], [0.7, 0.0], [0.5, 0.0],
            [0.5, 1.0], [0.3, 1.0], [0.7, 1.0]
        ])
        
        self.contours['J'] = self._scale_points([
            [0.3, 0.0], [0.7, 0.0], [0.6, 0.0], [0.6, 0.85],
            [0.5, 1.0], [0.3, 1.0], [0.2, 0.85]
        ])
        
        self.contours['K'] = self._scale_points([
            [0.2, 0.0], [0.2, 1.0], [0.2, 0.5],
            [0.8, 0.0], [0.2, 0.5], [0.8, 1.0]
        ])
        
        self.contours['L'] = self._scale_points([
            [0.2, 0.0], [0.2, 1.0], [0.8, 1.0]
        ])
        
        self.contours['M'] = self._scale_points([
            [0.15, 1.0], [0.15, 0.0], [0.5, 0.4],
            [0.85, 0.0], [0.85, 1.0]
        ])
        
        self.contours['N'] = self._scale_points([
            [0.2, 1.0], [0.2, 0.0], [0.8, 1.0], [0.8, 0.0]
        ])
        
        self.contours['O'] = self._scale_points([
            [0.3, 0.0], [0.15, 0.15], [0.15, 0.85], [0.3, 1.0],
            [0.7, 1.0], [0.85, 0.85], [0.85, 0.15], [0.7, 0.0], [0.3, 0.0]
        ])
        
        self.contours['P'] = self._scale_points([
            [0.2, 1.0], [0.2, 0.0], [0.7, 0.0], [0.8, 0.15],
            [0.8, 0.35], [0.7, 0.5], [0.2, 0.5]
        ])
        
        self.contours['Q'] = self._scale_points([
            [0.3, 0.0], [0.15, 0.15], [0.15, 0.85], [0.3, 1.0],
            [0.7, 1.0], [0.85, 0.85], [0.85, 0.15], [0.7, 0.0], [0.3, 0.0],
            [0.5, 0.5], [0.6, 0.7], [0.9, 1.1]
        ])
        
        self.contours['R'] = self._scale_points([
            [0.2, 1.0], [0.2, 0.0], [0.7, 0.0], [0.8, 0.15],
            [0.8, 0.35], [0.7, 0.5], [0.2, 0.5], [0.5, 0.5], [0.8, 1.0]
        ])
        
        self.contours['S'] = self._scale_points([
            [0.8, 0.15], [0.7, 0.0], [0.3, 0.0], [0.2, 0.15],
            [0.3, 0.35], [0.7, 0.5], [0.8, 0.65], [0.7, 1.0],
            [0.3, 1.0], [0.2, 0.85]
        ])
        
        self.contours['T'] = self._scale_points([
            [0.2, 0.0], [0.8, 0.0], [0.5, 0.0], [0.5, 1.0]
        ])
        
        self.contours['U'] = self._scale_points([
            [0.2, 0.0], [0.2, 0.85], [0.3, 1.0], [0.7, 1.0],
            [0.8, 0.85], [0.8, 0.0]
        ])
        
        self.contours['V'] = self._scale_points([
            [0.15, 0.0], [0.5, 1.0], [0.85, 0.0]
        ])
        
        self.contours['W'] = self._scale_points([
            [0.1, 0.0], [0.25, 1.0], [0.5, 0.6],
            [0.75, 1.0], [0.9, 0.0]
        ])
        
        self.contours['X'] = self._scale_points([
            [0.2, 0.0], [0.8, 1.0], [0.5, 0.5],
            [0.8, 0.0], [0.2, 1.0]
        ])
        
        self.contours['Y'] = self._scale_points([
            [0.2, 0.0], [0.5, 0.5], [0.8, 0.0],
            [0.5, 0.5], [0.5, 1.0]
        ])
        
        self.contours['Z'] = self._scale_points([
            [0.2, 0.0], [0.8, 0.0], [0.2, 1.0], [0.8, 1.0]
        ])
        
        # Lowercase letters (simplified versions)
        self.contours['a'] = self._scale_points([
            [0.8, 0.5], [0.8, 1.0], [0.7, 1.0], [0.3, 1.0], 
            [0.2, 0.9], [0.2, 0.7], [0.3, 0.6], [0.7, 0.6],
            [0.8, 0.7], [0.8, 1.0]
        ])
        
        self.contours['b'] = self._scale_points([
            [0.25, 0.0], [0.25, 1.0], [0.65, 1.0], [0.8, 0.85],
            [0.8, 0.65], [0.65, 0.5], [0.25, 0.5]
        ])
        
        self.contours['c'] = self._scale_points([
            [0.8, 0.6], [0.65, 0.5], [0.35, 0.5], [0.2, 0.65],
            [0.2, 0.85], [0.35, 1.0], [0.65, 1.0], [0.8, 0.9]
        ])
        
        self.contours['d'] = self._scale_points([
            [0.75, 0.0], [0.75, 1.0], [0.35, 1.0], [0.2, 0.85],
            [0.2, 0.65], [0.35, 0.5], [0.75, 0.5]
        ])
        
        self.contours['e'] = self._scale_points([
            [0.2, 0.75], [0.8, 0.75], [0.8, 0.65], [0.65, 0.5],
            [0.35, 0.5], [0.2, 0.65], [0.2, 0.85], [0.35, 1.0],
            [0.65, 1.0], [0.8, 0.9]
        ])
        
        self.contours['f'] = self._scale_points([
            [0.7, 0.1], [0.6, 0.0], [0.4, 0.0], [0.4, 0.5],
            [0.2, 0.5], [0.6, 0.5], [0.4, 0.5], [0.4, 1.0]
        ])
        
        self.contours['g'] = self._scale_points([
            [0.75, 0.5], [0.35, 0.5], [0.2, 0.65], [0.2, 0.85],
            [0.35, 1.0], [0.7, 1.0], [0.75, 0.95], [0.75, 1.1],
            [0.65, 1.2], [0.35, 1.2], [0.2, 1.05]
        ])
        
        self.contours['h'] = self._scale_points([
            [0.25, 0.0], [0.25, 1.0], [0.25, 0.65], [0.4, 0.5],
            [0.6, 0.5], [0.75, 0.65], [0.75, 1.0]
        ])
        
        self.contours['i'] = self._scale_points([
            [0.5, 0.5], [0.5, 1.0], [0.5, 0.25]
        ])
        
        self.contours['j'] = self._scale_points([
            [0.5, 0.5], [0.5, 1.1], [0.4, 1.2], [0.25, 1.15],
            [0.5, 0.25]
        ])
        
        self.contours['k'] = self._scale_points([
            [0.25, 0.0], [0.25, 1.0], [0.25, 0.7],
            [0.75, 0.5], [0.25, 0.7], [0.75, 1.0]
        ])
        
        self.contours['l'] = self._scale_points([
            [0.5, 0.0], [0.5, 1.0]
        ])
        
        self.contours['m'] = self._scale_points([
            [0.15, 1.0], [0.15, 0.65], [0.25, 0.5], [0.35, 0.5],
            [0.45, 0.6], [0.45, 1.0], [0.45, 0.6], [0.55, 0.5],
            [0.65, 0.5], [0.75, 0.6], [0.75, 1.0]
        ])
        
        self.contours['n'] = self._scale_points([
            [0.25, 1.0], [0.25, 0.65], [0.4, 0.5], [0.6, 0.5],
            [0.75, 0.65], [0.75, 1.0]
        ])
        
        self.contours['o'] = self._scale_points([
            [0.35, 0.5], [0.2, 0.65], [0.2, 0.85], [0.35, 1.0],
            [0.65, 1.0], [0.8, 0.85], [0.8, 0.65], [0.65, 0.5], [0.35, 0.5]
        ])
        
        self.contours['p'] = self._scale_points([
            [0.25, 0.5], [0.25, 1.2], [0.25, 1.0], [0.65, 1.0],
            [0.8, 0.85], [0.8, 0.65], [0.65, 0.5], [0.25, 0.5]
        ])
        
        self.contours['q'] = self._scale_points([
            [0.75, 0.5], [0.75, 1.2], [0.75, 1.0], [0.35, 1.0],
            [0.2, 0.85], [0.2, 0.65], [0.35, 0.5], [0.75, 0.5]
        ])
        
        self.contours['r'] = self._scale_points([
            [0.25, 1.0], [0.25, 0.65], [0.4, 0.5], [0.65, 0.5]
        ])
        
        self.contours['s'] = self._scale_points([
            [0.75, 0.6], [0.65, 0.5], [0.35, 0.5], [0.25, 0.6],
            [0.35, 0.7], [0.65, 0.8], [0.75, 0.9], [0.65, 1.0],
            [0.35, 1.0], [0.25, 0.9]
        ])
        
        self.contours['t'] = self._scale_points([
            [0.45, 0.2], [0.45, 0.5], [0.25, 0.5], [0.65, 0.5],
            [0.45, 0.5], [0.45, 0.95], [0.55, 1.0], [0.7, 1.0]
        ])
        
        self.contours['u'] = self._scale_points([
            [0.25, 0.5], [0.25, 0.85], [0.35, 1.0], [0.65, 1.0],
            [0.75, 0.9], [0.75, 0.5], [0.75, 1.0]
        ])
        
        self.contours['v'] = self._scale_points([
            [0.2, 0.5], [0.5, 1.0], [0.8, 0.5]
        ])
        
        self.contours['w'] = self._scale_points([
            [0.15, 0.5], [0.3, 1.0], [0.5, 0.75],
            [0.7, 1.0], [0.85, 0.5]
        ])
        
        self.contours['x'] = self._scale_points([
            [0.25, 0.5], [0.75, 1.0], [0.5, 0.75],
            [0.75, 0.5], [0.25, 1.0]
        ])
        
        self.contours['y'] = self._scale_points([
            [0.25, 0.5], [0.5, 0.9], [0.75, 0.5],
            [0.5, 0.9], [0.5, 1.2]
        ])
        
        self.contours['z'] = self._scale_points([
            [0.25, 0.5], [0.75, 0.5], [0.25, 1.0], [0.75, 1.0]
        ])
        
        # Numbers
        self.contours['0'] = self._scale_points([
            [0.3, 0.0], [0.15, 0.15], [0.15, 0.85], [0.3, 1.0],
            [0.7, 1.0], [0.85, 0.85], [0.85, 0.15], [0.7, 0.0], [0.3, 0.0]
        ])
        
        self.contours['1'] = self._scale_points([
            [0.4, 0.2], [0.5, 0.0], [0.5, 1.0],
            [0.3, 1.0], [0.7, 1.0]
        ])
        
        self.contours['2'] = self._scale_points([
            [0.2, 0.2], [0.3, 0.0], [0.7, 0.0], [0.8, 0.15],
            [0.8, 0.35], [0.2, 1.0], [0.8, 1.0]
        ])
        
        self.contours['3'] = self._scale_points([
            [0.2, 0.15], [0.3, 0.0], [0.7, 0.0], [0.8, 0.15],
            [0.7, 0.3], [0.5, 0.5], [0.7, 0.7], [0.8, 0.85],
            [0.7, 1.0], [0.3, 1.0], [0.2, 0.85]
        ])
        
        self.contours['4'] = self._scale_points([
            [0.65, 0.0], [0.25, 0.65], [0.8, 0.65],
            [0.65, 0.65], [0.65, 1.0]
        ])
        
        self.contours['5'] = self._scale_points([
            [0.75, 0.0], [0.25, 0.0], [0.25, 0.45], [0.65, 0.45],
            [0.8, 0.6], [0.8, 0.85], [0.7, 1.0], [0.3, 1.0], [0.2, 0.85]
        ])
        
        self.contours['6'] = self._scale_points([
            [0.75, 0.15], [0.6, 0.0], [0.3, 0.0], [0.15, 0.15],
            [0.15, 0.85], [0.3, 1.0], [0.65, 1.0], [0.8, 0.85],
            [0.8, 0.65], [0.65, 0.5], [0.3, 0.5], [0.15, 0.65]
        ])
        
        self.contours['7'] = self._scale_points([
            [0.2, 0.0], [0.8, 0.0], [0.4, 1.0]
        ])
        
        self.contours['8'] = self._scale_points([
            [0.3, 0.5], [0.2, 0.35], [0.2, 0.15], [0.3, 0.0],
            [0.7, 0.0], [0.8, 0.15], [0.8, 0.35], [0.7, 0.5],
            [0.3, 0.5], [0.2, 0.65], [0.2, 0.85], [0.3, 1.0],
            [0.7, 1.0], [0.8, 0.85], [0.8, 0.65], [0.7, 0.5]
        ])
        
        self.contours['9'] = self._scale_points([
            [0.85, 0.35], [0.7, 0.5], [0.35, 0.5], [0.2, 0.35],
            [0.2, 0.15], [0.35, 0.0], [0.7, 0.0], [0.85, 0.15],
            [0.85, 0.85], [0.7, 1.0], [0.4, 1.0], [0.25, 0.85]
        ])
    
    def get_contour(self, char, height, width, shift_right, shift_down):
        """
        Get the scaled contour for a specific character.
        
        Args:
            char: The character to get the contour for
            height: Desired output height
            width: Desired output width
            
        Returns:
            numpy array of shape (n, 1, 2) compatible with cv2.drawContours
            or None if character not found
        """
        if char not in self.contours:
            return None

        contour = self.contours[char].reshape(-1, 2)

        # Compute bounding box of original contour
        x_min, y_min = contour.min(axis=0)
        x_max, y_max = contour.max(axis=0)

        orig_width = x_max - x_min
        orig_height = y_max - y_min

        # Avoid division by zero
        if orig_width == 0 or orig_height == 0:
            return contour.reshape(-1, 1, 2)

        # Compute scale factors
        scale_x = width / orig_width
        scale_y = height / orig_height

        # Apply scaling and translation so it starts at (0,0)
        scaled = (contour - [x_min, y_min]) * [scale_x, scale_y]

        # Apply shifts
        scaled += [shift_right, shift_down]

        return scaled.reshape(-1, 1, 2).astype(np.int32)
    
    def draw_text(self, img, text, origin=(10, 100), spacing=None, 
                  color=(0, 0, 255), thickness=2):
        """
        Draw text on an image using the contour library.
        
        Args:
            img: The image to draw on
            text: The text string to draw
            origin: (x, y) starting position
            spacing: Space between characters (default: height * 0.3)
            color: Color of the contours (BGR)
            thickness: Thickness of the lines
            
        Returns:
            The modified image
        """
        if spacing is None:
            spacing = int(self.height * 0.3)
        
        x, y = origin
        
        for char in text:
            if char == ' ':
                x += spacing
                continue
                
            contour = self.get_contour(char)
            if contour is not None:
                # Offset contour to current position
                offset_contour = contour.copy()
                offset_contour[:, 0, 0] += x
                offset_contour[:, 0, 1] += y
                
                # Draw the contour
                cv2.polylines(img, [offset_contour], False, color, thickness)
                
                # Move to next character position
                x += int(self.height * 0.7)
        
        return img
    
    def get_all_characters(self):
        """Return a list of all available characters."""
        return sorted(self.contours.keys())

# Example usage
if __name__ == "__main__":
    # Create a blank image
    img = np.ones((800, 1600, 3), dtype=np.uint8) * 255
    
    # Initialize the library
    library = ContourLibrary(height=80)
    
    # Draw some text
    library.draw_text(img, "ABCDEFG", origin=(20, 100), color=(255, 0, 0), thickness=2)
    library.draw_text(img, "HIJKLMNO", origin=(20, 220), color=(0, 128, 255), thickness=2)
    library.draw_text(img, "PQRSTUVW", origin=(20, 340), color=(0, 200, 0), thickness=2)
    library.draw_text(img, "XYZ", origin=(20, 460), color=(128, 0, 255), thickness=2)
    library.draw_text(img, "abcdefg", origin=(700, 100), color=(0, 0, 255), thickness=2)
    library.draw_text(img, "hijklmn", origin=(700, 220), color=(255, 0, 255), thickness=2)
    library.draw_text(img, "opqrst", origin=(700, 340), color=(255, 0, 255), thickness=2)
    library.draw_text(img, "uvwxyz", origin=(700, 460), color=(255, 165, 0), thickness=2)
    library.draw_text(img, "0123456789", origin=(20, 580), color=(0, 0, 0), thickness=2)
    # Display the result
    cv2.imshow("Contour Text Library", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Save the image
    cv2.imwrite("contour_text_demo.png", img)
    
    # Print available characters
    print("Available characters:", ''.join(library.get_all_characters()))