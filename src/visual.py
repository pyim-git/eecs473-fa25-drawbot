import matplotlib.pyplot as plt
import re



def extract_points(gcode_file_path):
    """
    Extract only X,Y coordinates from G1 commands in G-code file
    
    Returns:
        list: List of (x, y) tuples from G1 commands
    """
    all_points = []
    g0_points = []
    
    with open(gcode_file_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            
            contour_match = re.search('Contour\s+(\d+)', line)
            if contour_match: 
                contour_num = contour_match.group(1)
                print(f"contour number: {contour_num}")
            
            # process all points
            if line.startswith('G1') or line.startswith('G0'):                    
                # Extract X and Y coordinates using regex
                x_match = re.search(r'X([-\d.]+)', line)
                y_match = re.search(r'Y([-\d.]+)', line)
                
                if x_match and y_match:
                    x = float(x_match.group(1))
                    y = float(y_match.group(1))
                    all_points.append((x, y))
                else:
                    print(f"Warning: G1 command without X,Y coordinates on line {line_num}: {line}")

            # Only process G0 commands 
            if line.startswith('G0'):
                # Extract X and Y coordinates using regex
                x_match = re.search(r'X([-\d.]+)', line)
                y_match = re.search(r'Y([-\d.]+)', line)
                
                if x_match and y_match:
                    x = float(x_match.group(1))
                    y = float(y_match.group(1))
                    g0_points.append((x, y))
                else:
                    print(f"Warning: G0 command without X,Y coordinates on line {line_num}: {line}")

    return all_points, g0_points


"""Visualize g-code"""
def visualize_g1_coordinates(gcode_file_path, output_file):

    # Extract all points
    all_points, g0_points = extract_points(gcode_file_path)
    if not all_points:
        print("No G1 coordinates found in the file")
        return []
    
    # Convert to separate x and y arrays
    x_all = [p[0] for p in all_points]
    y_all = [p[1] for p in all_points]

    x_g0 = [p[0] for p in g0_points]
    y_g0 = [p[1] for p in g0_points]
    

    # Create visualization
    plt.figure(figsize=(12, 10))

    # Connect all points in each contour 
    with open(gcode_file_path, 'r') as f:
        current_line = 1
        points = []

        for line_num, line in enumerate(f, 1):
            line = line.strip()
            
            contour_match = re.search('Contour\s+(\d+)', line)
            contour_num = 0
            if contour_match: 
                contour_num = int(contour_match.group(1))
                print(f"Contour number: {contour_num}")
            
            if line.startswith('G1') or line.startswith('G0'):                    
                # Extract X and Y coordinates using regex
                x_match = re.search(r'X([-\d.]+)', line)
                y_match = re.search(r'Y([-\d.]+)', line)
                
                if x_match and y_match:
                    x = float(x_match.group(1))
                    y = float(y_match.group(1))
                    points.append((x, y))
                else:
                    print(f"Warning: G1 command without X,Y coordinates on line {line_num}: {line}")

            # plot the contour
            if (contour_num > current_line) or line.startswith('M30'):
                x_vals = [p[0] for p in points]
                y_vals = [p[1] for p in points]

                plt.plot(x_vals, y_vals, 'b-', linewidth=2, alpha=0.7)
                #plt.plot(x_vals, y_vals, 'ro', markersize=4, alpha=0.7)
                current_line += 1
                points.clear()

    

    # # Plot G0 points (the starting point for each contour)
    #plt.plot(x_g0, y_g0, 'k^', markersize = 8, alpha = 0.7, label = 'Start Point of Contour (in order)')
    
    # Mark first and last point of robot path
    plt.plot(x_all[0], y_all[0], 'go', markersize=10, markeredgecolor='black', label='Start')
    plt.plot(x_all[-1], y_all[-1], 'gs', markersize=10, markeredgecolor='black', label='End')
    
    # Add point labels for all g0 points
    #for i, (x, y) in enumerate(g0_points):  
       # plt.annotate(f'P{i+1}', (x, y), xytext=(5, 5), textcoords='offset points', 
                   # fontsize=8, alpha=0.7)
    
    plt.gca().invert_yaxis() # flip the y axis
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title(f'Robot Drawing Path: {gcode_file_path}\n({len(all_points)} points)')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()
    
    plt.tight_layout()
    plt.savefig("../output/"f"{output_file}") 
    plt.show()

    
    # Print detailed information
    print(f"Found {len(all_points)} coordinates")
    print(f"X range: {min(x_vals):.2f} to {max(x_vals):.2f}")
    print(f"Y range: {min(y_vals):.2f} to {max(y_vals):.2f}")
    print(f"First point: X{x_vals[0]:.2f} Y{y_vals[0]:.2f}")
    print(f"Last point: X{x_vals[-1]:.2f} Y{y_vals[-1]:.2f}")
    
    return all_points

  
  