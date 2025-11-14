import matplotlib.pyplot as plt
import re

# Matplot available colors
color_map = {
    'blue' : 'b',
    'green': 'g',
    'red' : 'r',
    'yellow': 'yellow',
    'brown' : '#A52A2A',
    'orange' : 'orange',
    'black' : 'k',
    'white' : 'w',
    'purple': '#8F00FF',
    'pink': 'pink'
}


"""Visualize g-code"""
def visualize_gcode(gcode_file_path, output_folder, output_file, WIDTH, HEIGHT):

    # buffer to store the total points for stats
    all_points = []
    g0_points=[]
    
    # Create visualization
    plt.figure(figsize=(12, 10))

    # Connect all points in each contour 
    with open(gcode_file_path, 'r') as f:
        current_line = 1
        points = []
        color = 'b'

        for line_num, line in enumerate(f, 1):
            line = line.strip()
            
            contour_match = re.search('Contour\s+(\d+)', line)
            color_match = re.search('Color\s+(.+)', line)
            if contour_match: 
                contour_num = int(contour_match.group(1))

            if color_match:
                color = color_map[color_match.group(1)]
            
            if line.startswith('G1') or line.startswith('G0'):                    
                # Extract X and Y coordinates using regex
                x_match = re.search(r'X([-\d.]+)', line)
                y_match = re.search(r'Y([-\d.]+)', line)
                
                if x_match and y_match:
                    x = float(x_match.group(1))
                    y = float(y_match.group(1))
                    points.append((x, y))        # save points for current contour

                    all_points.append((x,y))     # save total points
                    if line.startswith('G0'):     
                        g0_points.append((x,y))  # save total g0 points
                    
                else:
                    print(f"Warning: G1 command without X,Y coordinates on line {line_num}: {line}")

            # plot the contour after parsing its endpoint
            if (contour_num > current_line) or line.startswith('M30'):
                x_vals = [p[0] for p in points]
                y_vals = [p[1] for p in points]

                plt.plot(x_vals, y_vals, color, linewidth=1, alpha=0.7)

                #plt.plot(x_vals, y_vals, 'ro', markersize=2, alpha=0.7)
                current_line += 1
                points.clear()

    # Convert to separate x and y arrays
    x_all = [p[0] for p in all_points]
    y_all = [p[1] for p in all_points]
    x_g0 = [p[0] for p in g0_points]
    y_g0 = [p[1] for p in g0_points]

    # Plot G0 points (the starting point for each contour)
   # plt.plot(x_g0, y_g0, 'k^', markersize = 2, alpha = 0.7, label = 'Start Point of Contour (in order)')
    
    # Mark first and last point of robot path
    plt.plot(x_all[0], y_all[0], 'go', markersize=10, markeredgecolor='black', label='Start')
    plt.plot(x_all[-1], y_all[-1], 'gs', markersize=10, markeredgecolor='black', label='End')
    
    # Add point labels for all g0 points
    # for i, (x, y) in enumerate(g0_points):  
    #     plt.annotate(f'P{i+1}', (x, y), xytext=(4, 4), textcoords='offset points', 
    #                 fontsize=8, alpha=0.7)
    
    plt.gca().invert_yaxis() # flip the y axis
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title(f'Robot Drawing Path: {len(all_points)} points)')
    plt.grid(True, alpha=0.3)
    plt.gca().set_aspect(HEIGHT/WIDTH)  # height/width


    plt.legend()    
    plt.tight_layout()
    plt.savefig(f"{output_folder}/{output_file}") 
    plt.show()

    
    return all_points

  
  