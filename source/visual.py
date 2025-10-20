import matplotlib.pyplot as plt
import re

def extract_g1_coordinates(gcode_file_path):
    """
    Extract only X,Y coordinates from G1 commands in G-code file
    
    Returns:
        list: List of (x, y) tuples from G1 commands
    """
    g1_points = []
    
    with open(gcode_file_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            
            # Only process G1 commands
            if line.startswith('G1') or line.startswith('G0'):
                # Extract X and Y coordinates using regex
                x_match = re.search(r'X([-\d.]+)', line)
                y_match = re.search(r'Y([-\d.]+)', line)
                
                if x_match and y_match:
                    x = float(x_match.group(1))
                    y = float(y_match.group(1))
                    g1_points.append((x, y))
                else:
                    print(f"Warning: G1 command without X,Y coordinates on line {line_num}: {line}")
    
    return g1_points



def visualize_g1_coordinates(gcode_file_path):
    """
    Visualize only coordinates from G1 commands
    """
    # Extract G1 coordinates
    points = extract_g1_coordinates(gcode_file_path)
    
    if not points:
        print("No G1 coordinates found in the file")
        return []
    
    # Create visualization
    plt.figure(figsize=(12, 10))
    
    # Convert to separate x and y arrays
    x_vals = [p[0] for p in points]
    y_vals = [p[1] for p in points]
    
    # Plot connected path
    plt.plot(x_vals, y_vals, 'b-', linewidth=2, alpha=0.7, label='G1 Path')
    
    # Plot points
    plt.plot(x_vals, y_vals, 'ro', markersize=4, alpha=0.7, label='G1 Points')
    
    # Mark first and last points
    plt.plot(x_vals[0], y_vals[0], 'go', markersize=10, markeredgecolor='black', label='Start (G1)')
    plt.plot(x_vals[-1], y_vals[-1], 'rs', markersize=10, markeredgecolor='black', label='End (G1)')
    
    # Add point labels for first few points
    for i, (x, y) in enumerate(points[:5]):  # Label first 5 points
        plt.annotate(f'P{i+1}', (x, y), xytext=(5, 5), textcoords='offset points', 
                    fontsize=8, alpha=0.7)
    
    plt.gca().invert_yaxis() # flip the y axis

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title(f'G1 Commands Visualization: {gcode_file_path}\n({len(points)} G1 points)')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()
    
    plt.tight_layout()
    plt.show()
    
    # Print detailed information
    print(f"Found {len(points)} G1 coordinates")
    print(f"X range: {min(x_vals):.2f} to {max(x_vals):.2f}")
    print(f"Y range: {min(y_vals):.2f} to {max(y_vals):.2f}")
    print(f"First point: X{x_vals[0]:.2f} Y{y_vals[0]:.2f}")
    print(f"Last point: X{x_vals[-1]:.2f} Y{y_vals[-1]:.2f}")
    
    return points

  
  