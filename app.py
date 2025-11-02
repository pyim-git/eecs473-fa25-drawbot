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

from flask import Flask, request, jsonify
from flask_cors import CORS
import os
from werkzeug.utils import secure_filename

app = Flask(__name__)
CORS(app)  # Enable CORS for browser requests

# Configure upload folder
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

def process_image(image_path, name):
    """
    Process the image - replace this with your actual processing logic.
    
    Args:
        image_path (str): Path to the uploaded image file
        name (str): Name of the snapshot
        
    Returns:
        dict: Processing result with status and output
    """
    # TODO: Add your image processing logic here
    # Examples:
    # - Convert to grayscale
    # - Apply edge detection
    # - Generate G-code for robot drawing
    # - etc.
    
    # Example placeholder processing:
    file_size = os.path.getsize(image_path)
    
    return {
        'success': True,
        'message': f'Image "{name}" processed successfully',
        'output': f'File size: {file_size} bytes\nPath: {image_path}',
        'file_size': file_size
    }

@app.route('/process', methods=['POST'])
def process():
    """Handle image processing requests"""
    try:
        # Check if image file is present
        if 'image' not in request.files:
            return jsonify({'error': 'No image file provided'}), 400
        
        file = request.files['image']
        name = request.form.get('name', 'unnamed')
        
        if file.filename == '':
            return jsonify({'error': 'No file selected'}), 400
        
        # Save the uploaded file
        filename = secure_filename('myimage.png')
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(filepath)
        
        # Process the image
        result = process_image(filepath, name)
        
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

if __name__ == '__main__':
    print('=' * 60)
    print('Local Image Processing API Server')
    print('=' * 60)
    print('Server running at: http://localhost:500')
    print('Process endpoint: http://localhost:500/process')
    print('Health check: http://localhost:500/health')
    print('=' * 60)
    print('\nReady to receive image processing requests!')
    print('Press Ctrl+C to stop the server\n')
    
    app.run(host='0.0.0.0', port=500, debug=True)
