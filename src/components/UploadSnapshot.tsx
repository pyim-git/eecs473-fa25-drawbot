import React, { useState, useRef } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from './ui/card';
import { Button } from './ui/button';
import { Input } from './ui/input';
import { Label } from './ui/label';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from './ui/select';
import { Progress } from './ui/progress';
import { Alert, AlertDescription } from './ui/alert';
import { Upload, Image, FileText, CheckCircle, X } from 'lucide-react';
import { Snapshot, useAuth } from '../App';
import { projectId, publicAnonKey } from '../utils/supabase/info';

export function UploadSnapshot() {
  const [selectedFile, setSelectedFile] = useState<File | null>(null);
  const [snapshotName, setSnapshotName] = useState('');
  const [canvasSize, setCanvasSize] = useState('');
  const [isUploading, setIsUploading] = useState(false);
  const [uploadProgress, setUploadProgress] = useState(0);
  const [uploadComplete, setUploadComplete] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const [error, setError] = useState('');
  const fileInputRef = useRef<HTMLInputElement>(null);
  const { accessToken } = useAuth();

  const handleFileSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      // Validate file type
      if (!file.type.startsWith('image/')) {
        setError('Please select an image file (JPG, PNG, GIF, etc.)');
        return;
      }
      
      // Validate file size (max 10MB)
      if (file.size > 10 * 1024 * 1024) {
        setError('File size must be less than 10MB');
        return;
      }

      setSelectedFile(file);
      setError('');
      
      // Auto-generate name if not set
      if (!snapshotName) {
        const nameWithoutExtension = file.name.replace(/\.[^/.]+$/, '');
        setSnapshotName(nameWithoutExtension);
      }
    }
  };

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
  };

  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    const file = e.dataTransfer.files[0];
    if (file) {
      if (!file.type.startsWith('image/')) {
        setError('Please select an image file (JPG, PNG, GIF, etc.)');
        return;
      }
      
      if (file.size > 10 * 1024 * 1024) {
        setError('File size must be less than 10MB');
        return;
      }

      setSelectedFile(file);
      setError('');
      
      if (!snapshotName) {
        const nameWithoutExtension = file.name.replace(/\.[^/.]+$/, '');
        setSnapshotName(nameWithoutExtension);
      }
    }
  };

  const uploadToSupabase = async () => {
    if (!selectedFile || !accessToken) return;
    
    setIsUploading(true);
    setUploadProgress(0);
    
    try {
      // Create form data
      const formData = new FormData();
      formData.append('file', selectedFile);
      formData.append('name', snapshotName);
      formData.append('canvasSize', canvasSize);
      
      // Start upload
      setUploadProgress(25);
      
      const response = await fetch(`https://${projectId}.supabase.co/functions/v1/make-server-2eb0fb94/snapshots/upload`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${accessToken}`,
        },
        body: formData,
      });
      
      setUploadProgress(75);
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Upload failed');
      }
      
      setUploadProgress(100);
      await new Promise(resolve => setTimeout(resolve, 500)); // Show 100% briefly
      
      setIsUploading(false);
      setUploadComplete(true);
    } catch (error) {
      console.log('Upload error:', error);
      setError(error instanceof Error ? error.message : 'Upload failed');
      setIsUploading(false);
      setUploadProgress(0);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!selectedFile || !snapshotName || !canvasSize) {
      setError('Please fill in all required fields');
      return;
    }
    
    if (!accessToken) {
      setError('You must be signed in to upload snapshots');
      return;
    }
    
    setError('');
    await uploadToSupabase();
  };

  const resetForm = () => {
    setSelectedFile(null);
    setSnapshotName('');
    setCanvasSize('');
    setUploadProgress(0);
    setUploadComplete(false);
    setIsProcessing(false);
    setError('');
    if (fileInputRef.current) {
      fileInputRef.current.value = '';
    }
  };

  const handleProcessImage = async () => {
    setIsProcessing(true);
    setError('');

    try {
      // Get the uploaded file blob
      if (!selectedFile) {
        setError('No image file selected');
        return;
      }

      // Create FormData to send to local API
      const formData = new FormData();
      formData.append('image', selectedFile);
      formData.append('name', snapshotName);

      // Try to connect to local Python API (default: http://localhost:5000/process)
      const localApiUrl = 'http://localhost:500/process';
      
      try {
        const response = await fetch(localApiUrl, {
          method: 'POST',
          body: formData,
        });

        if (!response.ok) {
          throw new Error(`Local API returned status ${response.status}`);
        }

        const result = await response.json();
        console.log('Processing result:', result);
        
        // Show success message with output
        let successMessage = 'Image processed successfully!';
        if (result.output || result.message) {
          successMessage += `\n\n${result.output || result.message}`;
        }
        alert(successMessage);
        setError('');
      } catch (fetchError) {
        console.error('Local API error:', fetchError);
        
        // If local API is not available, provide download option
        setError(
          'Could not connect to local processing API.\n\n' +
          'To process images locally:\n' +
          '1. Start your local Python API server (Flask/FastAPI)\n' +
          '2. Make sure it\'s running at http://localhost:500/process\n' +
          '3. Or click "Download Image" to process manually'
        );
      }
    } catch (error) {
      console.error('Processing error:', error);
      setError(error instanceof Error ? error.message : 'Processing failed');
    } finally {
      setIsProcessing(false);
    }
  };

  const handleDownloadImage = () => {
    if (!selectedFile) {
      setError('No image file to download');
      return;
    }

    // Create a download link
    const url = URL.createObjectURL(selectedFile);
    const link = document.createElement('a');
    link.href = url;
    link.download = selectedFile.name;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  if (uploadComplete) {
    return (
      <div className="max-w-2xl mx-auto space-y-6">
        <Card>
          <CardContent className="flex flex-col items-center justify-center py-12 space-y-4">
            <CheckCircle className="h-16 w-16 text-green-500 mb-4" />
            <h2 className="text-xl mb-2">Upload Complete!</h2>
            <p className="text-muted-foreground text-center mb-6">
              Your snapshot "{snapshotName}" has been saved successfully.
            </p>
            
            {error && (
              <Alert variant="destructive" className="w-full max-w-md">
                <AlertDescription className="whitespace-pre-wrap">{error}</AlertDescription>
              </Alert>
            )}
            
            <div className="flex flex-col gap-3 w-full max-w-md">
              <div className="flex gap-3">
                <Button 
                  onClick={handleProcessImage}
                  disabled={isProcessing}
                  variant="outline"
                  className="flex-1"
                >
                  {isProcessing ? 'Processing...' : 'Process Image'}
                </Button>
                <Button 
                  onClick={handleDownloadImage}
                  variant="outline"
                  className="flex-1"
                >
                  Download Image
                </Button>
              </div>
              <Button onClick={resetForm} className="w-full">
                Upload Another Snapshot
              </Button>
            </div>
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <div className="max-w-2xl mx-auto space-y-6">
      <div>
        <h1 className="text-3xl">Upload Snapshot</h1>
        <p className="text-muted-foreground">
          Upload a new drawing snapshot to your collection
        </p>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Snapshot Details</CardTitle>
        </CardHeader>
        <CardContent>
          <form onSubmit={handleSubmit} className="space-y-6">
            {/* File Upload Area */}
            <div className="space-y-2">
              <Label>Image File</Label>
              <div
                className="border-2 border-dashed border-muted-foreground/25 rounded-lg p-8 text-center hover:border-muted-foreground/50 transition-colors cursor-pointer"
                onDragOver={handleDragOver}
                onDrop={handleDrop}
                onClick={() => fileInputRef.current?.click()}
              >
                <input
                  ref={fileInputRef}
                  type="file"
                  accept="image/*"
                  onChange={handleFileSelect}
                  className="hidden"
                />
                
                {selectedFile ? (
                  <div className="space-y-2">
                    <Image className="h-8 w-8 mx-auto text-green-500" />
                    <p className="font-medium">{selectedFile.name}</p>
                    <p className="text-sm text-muted-foreground">
                      {(selectedFile.size / 1024 / 1024).toFixed(2)} MB
                    </p>
                    <Button
                      type="button"
                      variant="outline"
                      size="sm"
                      onClick={(e) => {
                        e.stopPropagation();
                        setSelectedFile(null);
                        if (fileInputRef.current) fileInputRef.current.value = '';
                      }}
                    >
                      <X className="h-3 w-3 mr-1" />
                      Remove
                    </Button>
                  </div>
                ) : (
                  <div className="space-y-2">
                    <Upload className="h-8 w-8 mx-auto text-muted-foreground" />
                    <p>Click to select or drag and drop an image</p>
                    <p className="text-sm text-muted-foreground">
                      Supports JPG, PNG, GIF up to 10MB
                    </p>
                  </div>
                )}
              </div>
            </div>

            {/* Snapshot Name */}
            <div className="space-y-2">
              <Label htmlFor="snapshot-name">Snapshot Name</Label>
              <Input
                id="snapshot-name"
                type="text"
                placeholder="Enter a name for your snapshot"
                value={snapshotName}
                onChange={(e) => setSnapshotName(e.target.value)}
                required
              />
            </div>

            {/* Canvas Size */}
            <div className="space-y-2">
              <Label htmlFor="canvas-size">Canvas Size</Label>
              <Select value={canvasSize} onValueChange={setCanvasSize} required>
                <SelectTrigger>
                  <SelectValue placeholder="Select canvas size" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="18x12 inches">18" × 12" (Small)</SelectItem>
                  <SelectItem value="24x18 inches">24" × 18" (Medium)</SelectItem>
                  <SelectItem value="36x24 inches">36" × 24" (Large)</SelectItem>
                  <SelectItem value="48x36 inches">48" × 36" (Extra Large)</SelectItem>
                  <SelectItem value="Custom">Custom Size</SelectItem>
                </SelectContent>
              </Select>
            </div>

            {error && (
              <Alert variant="destructive">
                <AlertDescription>{error}</AlertDescription>
              </Alert>
            )}

            {isUploading && (
              <div className="space-y-2">
                <div className="flex items-center justify-between text-sm">
                  <span>Uploading...</span>
                  <span>{uploadProgress}%</span>
                </div>
                <Progress value={uploadProgress} />
              </div>
            )}

            <Button
              type="submit"
              className="w-full"
              disabled={!selectedFile || !snapshotName || !canvasSize || isUploading}
            >
              {isUploading ? 'Uploading...' : 'Upload Snapshot'}
            </Button>
          </form>
        </CardContent>
      </Card>

      <Card>
        <CardContent className="p-4">
          <div className="flex items-start space-x-3">
            <FileText className="h-5 w-5 text-muted-foreground mt-0.5" />
            <div className="space-y-1">
              <p className="text-sm font-medium">Tips for better uploads:</p>
              <ul className="text-sm text-muted-foreground space-y-1">
                <li>• Use high-resolution images for best results</li>
                <li>• Ensure good lighting and minimal shadows</li>
                <li>• Crop images to show only the drawing area</li>
                <li>• Name your snapshots descriptively for easy organization</li>
              </ul>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}