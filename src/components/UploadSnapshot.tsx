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

type ColorType = 'black' | 'blue' | 'red' | 'green' | 'purple' | 'orange' | 'brown' | 'yellow';

const AVAILABLE_COLORS: ColorType[] = ['black', 'blue', 'red', 'green', 'purple', 'orange', 'brown', 'yellow'];

const COLOR_DISPLAY: Record<ColorType, { name: string; hex: string }> = {
  black: { name: 'Black', hex: '#000000' },
  blue: { name: 'Blue', hex: '#0000FF' },
  red: { name: 'Red', hex: '#FF0000' },
  green: { name: 'Green', hex: '#00FF00' },
  purple: { name: 'Purple', hex: '#800080' },
  orange: { name: 'Orange', hex: '#FFA500' },
  brown: { name: 'Brown', hex: '#A52A2A' },
  yellow: { name: 'Yellow', hex: '#FFFF00' },
};

export function UploadSnapshot() {
  const [selectedFile, setSelectedFile] = useState<File | null>(null);
  const [snapshotName, setSnapshotName] = useState('');
  const [canvasSize, setCanvasSize] = useState('1');
  const [isUploading, setIsUploading] = useState(false);
  const [uploadProgress, setUploadProgress] = useState(0);
  const [uploadComplete, setUploadComplete] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const [error, setError] = useState('');
  const [topLeftX, setTopLeftX] = useState('');
  const [topLeftY, setTopLeftY] = useState('');
  const [bottomRightX, setBottomRightX] = useState('');
  const [bottomRightY, setBottomRightY] = useState('');
  const [imageType, setImageType] = useState<'digital' | 'photo'>('digital');
  const [markerSelections, setMarkerSelections] = useState<{ markerColor: ColorType | null; imageColors: ColorType[] }[]>([
    { markerColor: null, imageColors: [] },
    { markerColor: null, imageColors: [] },
    { markerColor: null, imageColors: [] },
  ]);
  const [draggedColor, setDraggedColor] = useState<ColorType | null>(null);
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
      
      // Add marker color mappings
      markerSelections.forEach((selection, index) => {
        if (selection.markerColor) {
          formData.append(`markerColor${index + 1}`, selection.markerColor);
          // Add image colors mapped to this marker
          selection.imageColors.forEach((imageColor, imgIndex) => {
            formData.append(`marker${index + 1}_imageColor${imgIndex + 1}`, imageColor);
          });
        }
      });
      console.log(markerSelections);
      
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
    
    if (!selectedFile || !snapshotName ) {
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
    setCanvasSize('1');
    setUploadProgress(0);
    setUploadComplete(false);
    setIsProcessing(false);
    setError('');
    setTopLeftX('');
    setTopLeftY('');
    setBottomRightX('');
    setBottomRightY('');
    setMarkerSelections([
      { markerColor: null, imageColors: [] },
      { markerColor: null, imageColors: [] },
      { markerColor: null, imageColors: [] },
    ]);
    if (fileInputRef.current) {
      fileInputRef.current.value = '';
    }
  };

  const handleColorDragStart = (e: React.DragEvent, color: ColorType) => {
    setDraggedColor(color);
    e.dataTransfer.effectAllowed = 'move';
  };

  const handleMarkerSlotDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'move';
  };

  const handleMarkerSlotDrop = (e: React.DragEvent, slotIndex: number) => {
    e.preventDefault();
    if (draggedColor) {
      const newMarkerSelections = [...markerSelections];
      const currentSelection = newMarkerSelections[slotIndex];
      
      // Add color to this slot if not already present
      if (!currentSelection.imageColors.includes(draggedColor)) {
        currentSelection.imageColors.push(draggedColor);
      }
      
      setMarkerSelections(newMarkerSelections);
      setDraggedColor(null);
    }
  };

  const handleRemoveImageColor = (slotIndex: number, colorToRemove: ColorType) => {
    const newMarkerSelections = [...markerSelections];
    newMarkerSelections[slotIndex].imageColors = newMarkerSelections[slotIndex].imageColors.filter(
      (color) => color !== colorToRemove
    );
    setMarkerSelections(newMarkerSelections);
  };

  const handleMarkerColorChange = (slotIndex: number, markerColor: ColorType | null) => {
    const newMarkerSelections = [...markerSelections];
    newMarkerSelections[slotIndex].markerColor = markerColor;
    setMarkerSelections(newMarkerSelections);
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

      // Calculate width and height from corner coordinates
      const topLeftXNum = parseFloat(topLeftX);
      const topLeftYNum = parseFloat(topLeftY);
      const bottomRightXNum = parseFloat(bottomRightX);
      const bottomRightYNum = parseFloat(bottomRightY);

      if (isNaN(topLeftXNum) || isNaN(topLeftYNum) || isNaN(bottomRightXNum) || isNaN(bottomRightYNum)) {
        setError('Please enter valid corner coordinates');
        return;
      }

      const widthMM = Math.abs(bottomRightXNum - topLeftXNum);
      const heightMM = Math.abs(bottomRightYNum - topLeftYNum);

      if (widthMM <= 0 || heightMM <= 0) {
        setError('Invalid coordinates: width and height must be greater than 0');
        return;
      }

      // Create FormData to send to local API
      const formData = new FormData();
      formData.append('image', selectedFile);
      formData.append('name', snapshotName);
      formData.append('imageType', imageType);
      formData.append('widthMM', widthMM.toString());
      formData.append('heightMM', heightMM.toString());
      
      // Add marker color mappings
      markerSelections.forEach((selection) => {
        if (selection.markerColor) {
          formData.append(`markerColor`, selection.markerColor);
          // Add image colors mapped to this marker
          selection.imageColors.forEach((imageColor) => {
            formData.append(`mapping`, imageColor);
          });
        }
      });
      console.log(markerSelections);

      const localApiUrl = 'http://localhost:500/process/';
      
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
        setError('Could not connect to local processing API.');
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
            {/* <div className="space-y-2">
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
            </div> */}

            {/* Digital or Photo */}
            <div className="space-y-2">
              <Label htmlFor="image-type">Image Type</Label>
              <Select value={imageType} onValueChange={setImageType} required>
                <SelectTrigger>
                  <SelectValue placeholder="Select image type" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="digital">Digital</SelectItem>
                  <SelectItem value="photo">Photo</SelectItem>
                </SelectContent>
              </Select>
            </div>

            {/* Corner Coordinates */}
            <div className="space-y-4">
              <Label>Corner Coordinates (mm)</Label>
              
              {/* Top Left Corner */}
              <div className="space-y-2">
                <Label className="text-sm text-muted-foreground">Top Left Corner</Label>
                <div className="grid grid-cols-2 gap-3">
                  <div className="space-y-2">
                    <Label htmlFor="top-left-x" className="text-xs">X</Label>
                    <Input
                      id="top-left-x"
                      type="number"
                      placeholder="X coordinate"
                      value={topLeftX}
                      onChange={(e) => setTopLeftX(e.target.value)}
                      required
                    />
                  </div>
                  <div className="space-y-2">
                    <Label htmlFor="top-left-y" className="text-xs">Y</Label>
                    <Input
                      id="top-left-y"
                      type="number"
                      placeholder="Y coordinate"
                      value={topLeftY}
                      onChange={(e) => setTopLeftY(e.target.value)}
                      required
                    />
                  </div>
                </div>
              </div>

              {/* Bottom Right Corner */}
              <div className="space-y-2">
                <Label className="text-sm text-muted-foreground">Bottom Right Corner</Label>
                <div className="grid grid-cols-2 gap-3">
                  <div className="space-y-2">
                    <Label htmlFor="bottom-right-x" className="text-xs">X</Label>
                    <Input
                      id="bottom-right-x"
                      type="number"
                      placeholder="X coordinate"
                      value={bottomRightX}
                      onChange={(e) => setBottomRightX(e.target.value)}
                      required
                    />
                  </div>
                  <div className="space-y-2">
                    <Label htmlFor="bottom-right-y" className="text-xs">Y</Label>
                    <Input
                      id="bottom-right-y"
                      type="number"
                      placeholder="Y coordinate"
                      value={bottomRightY}
                      onChange={(e) => setBottomRightY(e.target.value)}
                      required
                    />
                  </div>
                </div>
              </div>
            </div>

            {/* Marker Color Mapping */}
            <div className="space-y-4">
              <Label>Map Image Colors to Marker Colors</Label>
              
              {/* Available Image Color Tiles */}
              <div className="space-y-2">
                <Label className="text-sm text-muted-foreground">Image Colors (Drag to markers below)</Label>
                <div className="flex flex-wrap gap-2">
                  {AVAILABLE_COLORS.map((color) => {
                    // Check if this color is already used in any marker
                    const isUsed = markerSelections.some(selection => 
                      selection.imageColors.includes(color)
                    );
                    return (
                      <div
                        key={color}
                        draggable
                        onDragStart={(e) => handleColorDragStart(e, color)}
                        className={`
                          px-3 py-2 rounded-lg border-2 transition-all bg-background
                          ${isUsed
                            ? 'border-muted-foreground/10 opacity-50 cursor-not-allowed'
                            : 'border-muted-foreground/25 hover:border-muted-foreground/50 hover:scale-105 cursor-move'
                          }
                        `}
                        title={isUsed 
                          ? `${COLOR_DISPLAY[color].name} is already assigned to a marker` 
                          : `Drag ${COLOR_DISPLAY[color].name} to a marker`
                        }
                      >
                        <span className={`text-sm font-medium ${isUsed ? 'text-muted-foreground' : ''}`}>
                          {COLOR_DISPLAY[color].name}
                        </span>
                      </div>
                    );
                  })}
                </div>
              </div>

              {/* Marker Color Slots */}
              <div className="space-y-2">
                <Label className="text-sm text-muted-foreground">Marker Colors</Label>
                <div className="grid grid-cols-1 gap-4">
                  {[0, 1, 2].map((slotIndex) => {
                    const selection = markerSelections[slotIndex];
                    return (
                      <div
                        key={slotIndex}
                        className="rounded-lg border-2 border-muted-foreground/25 p-4 space-y-3"
                      >
                        <div className="flex items-center gap-3">
                          <Label className="text-sm font-medium min-w-[100px]">Marker {slotIndex + 1}:</Label>
                          <Select
                            value={selection.markerColor || ''}
                            onValueChange={(value) => handleMarkerColorChange(slotIndex, value as ColorType | null)}
                          >
                            <SelectTrigger className="w-[180px]">
                              <SelectValue placeholder="Select marker color" />
                            </SelectTrigger>
                            <SelectContent>
                              {AVAILABLE_COLORS.map((color) => (
                                <SelectItem key={color} value={color}>
                                  {COLOR_DISPLAY[color].name}
                                </SelectItem>
                              ))}
                            </SelectContent>
                          </Select>
                        </div>
                        
                        <div
                          onDragOver={handleMarkerSlotDragOver}
                          onDrop={(e) => handleMarkerSlotDrop(e, slotIndex)}
                          className={`
                            min-h-[60px] rounded-lg border-2 border-dashed transition-all p-2
                            ${selection.imageColors.length > 0
                              ? 'border-muted-foreground/50 bg-muted/10'
                              : 'border-muted-foreground/25 hover:border-muted-foreground/50'
                            }
                          `}
                        >
                          {selection.imageColors.length > 0 ? (
                            <div className="flex flex-wrap gap-2">
                              {selection.imageColors.map((imageColor) => (
                                <div
                                  key={imageColor}
                                  className="px-3 py-1.5 rounded-md border border-muted-foreground/30 bg-background flex items-center gap-2 group"
                                >
                                  <span className="text-sm">{COLOR_DISPLAY[imageColor].name}</span>
                                  <Button
                                    type="button"
                                    variant="ghost"
                                    size="sm"
                                    className="h-4 w-4 p-0 hover:bg-destructive hover:text-destructive-foreground opacity-0 group-hover:opacity-100 transition-opacity"
                                    onClick={() => handleRemoveImageColor(slotIndex, imageColor)}
                                  >
                                    <X className="h-3 w-3" />
                                  </Button>
                                </div>
                              ))}
                            </div>
                          ) : (
                            <div className="flex items-center justify-center h-full min-h-[40px] text-muted-foreground text-sm">
                              Drop image colors here
                            </div>
                          )}
                        </div>
                      </div>
                    );
                  })}
                </div>
              </div>
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
              disabled={!selectedFile || !snapshotName  || isUploading}
            >
              {isUploading ? 'Uploading...' : 'Upload Snapshot'}
            </Button>
          </form>
        </CardContent>
      </Card>
    </div>
  );
}