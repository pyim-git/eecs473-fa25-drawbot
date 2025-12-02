#!/usr/bin/env python3
"""
Local Flask API Server for Image Processing

This is a simple Flask server that runs on your local machine (localhost:500)
to process images from the web application.

Setup:
1. Install Flask: pip install flask flask-cors pillow
2. Run this script: python3 local_api_server.py
3. The API will be available at http://localhost:500/process

The web app will send images to this endpoint for processing.
"""

from flask import Flask, request, jsonify, Response
from flask_cors import CORS
import os
from werkzeug.utils import secure_filename
from driver import *
from robot_detection import generate_frames, get_robo_loc

app = Flask(__name__)
CORS(app)  # Enable CORS for browser requests

# Configure upload folder
UPLOAD_FOLDER = 'backend/uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

def process_image(image_path, name, image_type, width_mm, height_mm, color_mappings):
    """
    Process the image - replace this with your actual processing logic.
    
    Args:
        image_path (str): Path to the uploaded image file
        name (str): Name of the snapshot
        image_type (str): Type of image (digital or photo)
        width_mm (int): Width in millimeters
        height_mm (int): Height in millimeters
        color_mappings (dict): Dictionary mapping marker colors to image colors
            Format: {marker_color: [image_color1, image_color2, ...], ...}
        
    Returns:
        dict: Processing result with status and output
    """
    print('hi')
    print(width_mm, height_mm, image_type, color_mappings)
    converter = GcodeConverter(width_mm, height_mm, image_type, color_mappings)
    print('hi')
    # Print color mappings if provided
    input_folder = "backend/uploads"
    output_folder = "backend/color_output"
    gcode_file1 = name+ "1.gcode" #stepper motor commands
    gcode_file2 = name+ "2.gcode" #gear motor commands
    gcode_plotfile = name+ ".png"
    print(f"{output_folder}/{gcode_file1}")
    output_file1, output_file2 = converter.image_to_gcode(image_path, f"{output_folder}/{gcode_file1}", f"{output_folder}/{gcode_file2}" )
    print(f"G-code saved to: {output_file1} and {output_file2}")
    print(f"G-code figure saved to: ", f"{output_folder}/{gcode_plotfile}")
    visualize_gcode(output_file1, f"{output_folder}", gcode_plotfile, width_mm, height_mm)
    file_size = os.path.getsize(image_path)
    
    return {
        'success': True,
        'message': f'Image "{name}" processed successfully',
        'output': f'File size: {file_size} bytes\nPath: {image_path}',
        'file_size': file_size
    }

@app.route('/process/', methods=['POST'])
def process():
    """Handle image processing requests"""
    try:
        # Check if image file is present
        if 'image' not in request.files:
            return jsonify({'error': 'No image file provided'}), 400
        
        file = request.files['image']
        name = request.form.get('name', 'unnamed')
        image_type = request.form.get('imageType')
        width_mm = request.form.get('widthMM')
        height_mm = request.form.get('heightMM')
        print(f"Processing image: {name}, {image_type}, {width_mm}, {height_mm}")
        markerColor = None
        color_mappings = dict()
        for key in request.form:
            if key.startswith("markerColor"):
                markerColor = request.form.get(key)
            elif key.startswith("mapping"):
                image_color = request.form.get(key)
                color_mappings[image_color] = markerColor
            print(color_mappings)
        print(color_mappings)
        if file.filename == '':
            return jsonify({'error': 'No file selected'}), 400
        
        # Save the uploaded file
        # filename = secure_filename('myimage.png')
        filename = secure_filename(name+'.png')
        app.config['UPLOAD_FOLDER'] = 'backend/uploads'
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(filepath)
        
        # Process the image
        result = process_image(filepath, name, image_type, int(width_mm), int(height_mm), color_mappings)
        
        return jsonify(result), 200
        
    except Exception as e:
        return jsonify({
            'error': 'Processing failed',
            'details': str(e)
        }), 500

@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({'status': 'ok', 'message': 'Local API server is running'}), 200

@app.route('/images', methods=['GET'])
def list_images():
    """List all available processed images in the color_output directory"""
    try:
        output_folder = 'backend/color_output'
        print('fetching images')
        if not os.path.exists(output_folder):
            return jsonify({'images': []}), 200
        
        # Get all PNG files (processed images)
        image_files = []
        for filename in os.listdir(output_folder):
            if filename.endswith('.png'):
                # Extract the base name (without extension) for the image name
                image_name = os.path.splitext(filename)[0]
                image_files.append({
                    'name': image_name,
                    'filename': filename,
                    'path': f'{output_folder}/{filename}'
                })
        
        # Sort by filename for consistent ordering
        image_files.sort(key=lambda x: x['filename'])
        
        return jsonify({'images': image_files}), 200
        
    except Exception as e:
        return jsonify({
            'error': 'Failed to list images',
            'details': str(e)
        }), 500

@app.route('/video_feed')
def video_feed():
    """
    Video streaming route that streams processed video frames with ArUco detection.
    Returns MJPEG stream that can be used in an HTML img tag or video element.
    """
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/robot_pos')
def get_robo_pos():
    """
    Return current robot position {'x': x, 'y': y}
    """
    return get_robo_loc()

if __name__ == '__main__':
    print('=' * 60)
    print('Local Image Processing API Server')
    print('=' * 60)
    print('Server running at: http://localhost:500')
    print('=' * 60)
    print('\nReady to receive image processing requests!')
    print('Press Ctrl+C to stop the server\n')
    
    app.run(host='0.0.0.0', port=500, debug=True)
